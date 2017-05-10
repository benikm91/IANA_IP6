#!/usr/bin/env python
import rospy

import time

from person_detection.config.production import iana_config
from person_detection.PersonDetection import PersonDetection


from person_detection.classifier.EuclideanDistanceClassifier import EuclideanDistanceClassifier

# Create all tables (if not exists)
from person_detection.clusterer.AverageFaceClusterer import AverageFaceClusterer
from person_detection.data_access.FaceFeatureDataAccessSQLAlchemy import FaceFeatureDataAccessSQLAlchemy
from person_detection.data_access.PersonDataAccessSQLAlchemy import PersonDataAccessSQLAlchemy
from person_detection.data_access.table_object.Base import Base
from person_detection.data_access.table_object.SQLLiteInMemory import engine_instance, session_maker
from person_detection.face_alignment.InnerEyesBottomLipFaceAlignment import InnerEyesBottomLipFaceAlignment
from person_detection.face_detection.DlibFaceDetection import DlibFaceDetector
from person_detection.face_recognition.embedding.OpenFaceEmbedder import OpenFaceEmbedder
from person_detection.face_recognition.labeling.FaceFilter import FaceFilter
from person_detection.face_recognition.labeling.FaceLabeler import FaceLabeler
from person_detection.face_recognition.labeling.UnknownFaceLabeler import UnknownFaceLabeler
from person_detection.grouping.FaceGrouper import FaceGrouper
from person_detection.grouping.SessionMemory import SessionMemory
from iana_person_detection.msg import KnownPersonEntered, UnknownPersonEntered

Base.metadata.create_all(engine_instance)

face_feature_data_access = FaceFeatureDataAccessSQLAlchemy(engine_instance, session_maker)
person_data_access = PersonDataAccessSQLAlchemy(engine_instance, session_maker)

face_data = face_feature_data_access.get_all()
person_data = person_data_access.get_all()

classifier = EuclideanDistanceClassifier()
classifier.train(face_data)

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

def detect_person(image_message):
    image_message.encoding = "bgr8"
    frame = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
    pd.detect_person(frame, time.time())


if __name__ == '__main__':
    try:
        rospy.init_node('person_detection', anonymous=True)
        rospy.Subscriber("/image", Image, detect_person)
        rospy.spin()
    except rospy.ROSInterruptException: pass