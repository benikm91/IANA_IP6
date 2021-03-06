#!/usr/bin/env python
from multiprocessing import Lock
import cv2

if __name__ == '__main__':

    import rospy

    import time
    import message_filters

    from iana_person_data.msg import Person
    from iana_person_data.srv import GetAllPersons, UpdatePersonFeatures
    from person_detection.config.production import iana_config
    from person_detection.PersonDetection import PersonDetection


    from person_detection.classifier.EuclideanDistanceClassifier import EuclideanDistanceClassifier

    # Create all tables (if not exists)
    from person_detection.clusterer.AverageFaceClusterer import AverageFaceClusterer
    from person_detection.face_alignment.InnerEyesBottomLipFaceAlignment import InnerEyesBottomLipFaceAlignment
    from person_detection.face_detection.DlibFaceDetection import DlibFaceDetector
    from person_detection.face_detection.HaarFaceDetection import HaarFaceDetector
    from person_detection.face_recognition.embedding.OpenFaceEmbedder import OpenFaceEmbedder
    from person_detection.face_recognition.labeling.FaceFilter import FaceFilter
    from person_detection.face_recognition.labeling.FaceLabeler import FaceLabeler
    from person_detection.face_recognition.labeling.UnknownFaceLabeler import UnknownFaceLabeler
    from person_detection.grouping.FaceGrouper import FaceGrouper
    from person_detection.grouping.SessionMemory import SessionMemory
    from iana_person_detection.msg import KnownPersonEntered, UnknownPersonEntered, UnknownPersonLeft, KnownPersonLeft, FaceBoundingBoxes

    from person_detection.cache.PersonCache import PersonCache

    # face_feature_data_access = FaceFeatureDataAccessSQLAlchemy(engine_instance, session_maker)
    get_persons = rospy.ServiceProxy('get_all_persons', GetAllPersons)

    if not get_persons.wait_for_service():
        rospy.logerr("Could not reach get_all_persons Service.")

    persons = get_persons().persons # type: list
    person_cache = PersonCache()

    for person in persons:
        person_cache.insert(person)

    classifier = EuclideanDistanceClassifier()
    classifier.train(persons)

    clusterer = AverageFaceClusterer(threshold_same=iana_config.clusterer.threshold_same)

    # detector = DlibFaceDetector(iana_config.face_detection.predictor_path, iana_config.face_detection.img_dim)
    detector = HaarFaceDetector(iana_config.haar_face_detection.model)
    embedder = OpenFaceEmbedder(iana_config.face_embedder.network_model, iana_config.face_embedder.img_dim, iana_config.face_embedder.cuda)
    alignment = InnerEyesBottomLipFaceAlignment(iana_config.dlib_face_detection.predictor_path, iana_config.dlib_face_detection.img_dim)
    faceLabeler = FaceLabeler(classifier)
    faceFilter = FaceFilter(iana_config.filter.threshold_known, iana_config.filter.threshold_unknown)
    faceGrouper = FaceGrouper(known_threshold=2, unknown_threshold=5, memory_span_known=10, memory_span_unknown=10)
    sessionMemory = SessionMemory()
    unknownFaceLabeler = UnknownFaceLabeler(clusterer)

    pd = PersonDetection(
        face_detection=detector,
        face_alignment=alignment,
        face_embedder=embedder,
        face_labeler=faceLabeler,
        face_filter=faceFilter,
        unknown_face_labeler=unknownFaceLabeler,
        face_grouper=faceGrouper,
        session_memory=sessionMemory,
        faces_detected_publisher=rospy.Publisher('/iana/faces_detected', FaceBoundingBoxes, queue_size=10),
        known_person_publisher=rospy.Publisher('/iana/person_detection/known/entered', KnownPersonEntered, queue_size=10),
        unknown_person_publisher=rospy.Publisher('/iana/person_detection/unknown/entered', UnknownPersonEntered, queue_size=10),
        update_person_features_service=rospy.ServiceProxy('/update_person_features', UpdatePersonFeatures),
        person_cache=person_cache,
        min_confidence_update=rospy.get_param('min_confidence_update', 0.9),
        person_max_feature_vector_count=rospy.get_param('person_max_feature_vector_count', 30)
    )

    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    bridge = CvBridge()

    lock = Lock()

    def detect_person(face_detection_image_message, person_detection_image_message):

        def get_image(message, encoding):
            message.encoding = encoding
            frame = bridge.imgmsg_to_cv2(message, desired_encoding=encoding)
            return frame

        age = rospy.get_rostime() - face_detection_image_message.header.stamp
        if age.to_sec() > rospy.get_param('~ignore_images_older_than', 1):
            return

        with lock:
            face_detection_image = get_image(face_detection_image_message, "bgr8")
            person_detection_image = get_image(person_detection_image_message, "bgr8")

            start_x, end_x = rospy.get_param('start_x', -1), rospy.get_param('end_x', 700)
            start_y, end_y = rospy.get_param('start_y', 0), rospy.get_param('end_y', 300)

            scale_factor = rospy.get_param('~scale_factor', 1)

            if start_x != -1:
                face_detection_image = face_detection_image[start_y:end_y, start_x:end_x]
            face_detection_image = cv2.resize(face_detection_image, (0, 0), fx=scale_factor, fy=scale_factor)

            pd.detect_person(face_detection_image, person_detection_image, face_detection_image_message.header.stamp.to_sec())

    def insert_new_person(person):
        person_cache.insert(person)
        with lock:
            classifier.update(person.person_id, map(lambda x: x.face, person.face_vectors))

    def update_person(person):
        person_cache.insert(person)
        with lock:
            classifier.update(person.person_id, map(lambda x: x.face, person.face_vectors))

    known_person_left_publisher = rospy.Publisher('/iana/person_detection/known/left', KnownPersonLeft, queue_size=10)
    unknown_person_left_publisher = rospy.Publisher('/iana/person_detection/unknown/left', UnknownPersonLeft, queue_size=10)

    try:
        rospy.init_node('person_detection', anonymous=True)
        face_detection_image = message_filters.Subscriber("/face_image", Image, queue_size=1)
        person_detection_image = message_filters.Subscriber("/person_image", Image, queue_size=1)

        message_filters.TimeSynchronizer([face_detection_image, person_detection_image], queue_size=1).registerCallback(detect_person)

        rospy.Subscriber("/new_person", Person, insert_new_person)
        rospy.Subscriber("/updated_person", Person, update_person)

        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            for known_left_id, _ in sessionMemory.known_remove_old():
                rospy.loginfo("Left: Known person id={0}".format(known_left_id))
                known_person_left_publisher.publish(person_cache[known_left_id])
            for unknown_left_id, _ in sessionMemory.unknown_remove_old():
                rospy.loginfo("Left: Unknown person id={0}".format(unknown_left_id))
                unknown_person_left_publisher.publish(unknown_left_id)
            r.sleep()

    except rospy.ROSInterruptException: pass
