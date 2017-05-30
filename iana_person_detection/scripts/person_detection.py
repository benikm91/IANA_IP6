#!/usr/bin/env python
from multiprocessing import Lock
import cv2

if __name__ == '__main__':

    import rospy

    import time

    from iana_person_data.msg import Person
    from iana_person_data.srv import GetAllPersons
    from person_detection.config.production import iana_config
    from person_detection.PersonDetection import PersonDetection


    from person_detection.classifier.EuclideanDistanceClassifier import EuclideanDistanceClassifier

    # Create all tables (if not exists)
    from person_detection.clusterer.AverageFaceClusterer import AverageFaceClusterer
    from person_detection.face_alignment.InnerEyesBottomLipFaceAlignment import InnerEyesBottomLipFaceAlignment
    from person_detection.face_detection.DlibFaceDetection import DlibFaceDetector
    from person_detection.face_recognition.embedding.OpenFaceEmbedder import OpenFaceEmbedder
    from person_detection.face_recognition.labeling.FaceFilter import FaceFilter
    from person_detection.face_recognition.labeling.FaceLabeler import FaceLabeler
    from person_detection.face_recognition.labeling.UnknownFaceLabeler import UnknownFaceLabeler
    from person_detection.grouping.FaceGrouper import FaceGrouper
    from person_detection.grouping.SessionMemory import SessionMemory
    from iana_person_detection.msg import KnownPersonEntered, UnknownPersonEntered, UnknownPersonLeft, KnownPersonLeft

    # face_feature_data_access = FaceFeatureDataAccessSQLAlchemy(engine_instance, session_maker)
    get_persons = rospy.ServiceProxy('get_all_persons', GetAllPersons)

    classifier = EuclideanDistanceClassifier()
    classifier.train(get_persons().persons)

    clusterer = AverageFaceClusterer(threshold_same=iana_config.clusterer.threshold_same)

    detector = DlibFaceDetector(iana_config.face_detection.predictor_path, iana_config.face_detection.img_dim)
    embedder = OpenFaceEmbedder(iana_config.face_embedder.network_model, iana_config.face_embedder.img_dim, iana_config.face_embedder.cuda)
    alignment = InnerEyesBottomLipFaceAlignment(iana_config.face_detection.predictor_path, iana_config.face_detection.img_dim)
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
        known_person_publisher=rospy.Publisher('/iana/person_detection/known/entered', KnownPersonEntered, queue_size=10),
        unknown_person_publisher=rospy.Publisher('/iana/person_detection/unknown/entered', UnknownPersonEntered, queue_size=10)
    )

    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    bridge = CvBridge()

    lock = Lock()

    def detect_person(image_message):
        # TODO Find face in low res image, but cut it out in high res image + landmark detection
        with lock:
            image_message.encoding = "bgr8" #TODO use "mono8"
            frame = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
            frame = cv2.resize(frame, (0, 0), fx=0.2, fy=0.2)
	    # TODO take time from image recording time
            pd.detect_person(frame, time.time())


    def insert_new_person(person):
        with lock:
            classifier.update(person.person_id, map(lambda x: x.face, person.face_vectors))

    known_person_left_publisher = rospy.Publisher('/iana/person_detection/known/left', KnownPersonLeft, queue_size=10)
    unknown_person_left_publisher = rospy.Publisher('/iana/person_detection/unknown/left', UnknownPersonLeft, queue_size=10)

    try:
        rospy.init_node('person_detection', anonymous=True)
        rospy.Subscriber("/image", Image, detect_person, queue_size=1)
        rospy.Subscriber("/new_person", Person, insert_new_person)

        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            for known_left_id, _ in sessionMemory.known_remove_old():
                rospy.loginfo("Left: Known person id={0}".format(known_left_id))
                known_person_left_publisher.publish(known_left_id)
            for unknown_left_id, _ in sessionMemory.unknown_remove_old():
                rospy.loginfo("Left: Unknown person id={0}".format(unknown_left_id))
                unknown_person_left_publisher.publish(unknown_left_id)

            r.sleep()

    except rospy.ROSInterruptException: pass
