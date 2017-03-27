import cv2
import time

from config.development import iana_config
from PersonDetection import PersonDetection


from classifier.EuclideanDistanceClassifier import EuclideanDistanceClassifier

# Create all tables (if not exists)
from clusterer.AverageFaceClusterer import AverageFaceClusterer
from data_access.FaceFeatureDataAccessSQLAlchemy import FaceFeatureDataAccessSQLAlchemy
from data_access.PersonDataAccessSQLAlchemy import PersonDataAccessSQLAlchemy
from data_access.table_object.Base import Base
from data_access.table_object.SQLLiteInMemory import engine_instance, session_maker
from face_alignment.InnerEyesBottomLipFaceAlignment import InnerEyesBottomLipFaceAlignment
from face_detection.DlibFaceDetection import DlibFaceDetector
from face_recognition.embedding.OpenFaceEmbedder import OpenFaceEmbedder
from face_recognition.labeling.FaceFilter import FaceFilter
from face_recognition.labeling.FaceLabeler import FaceLabeler
from face_recognition.labeling.UnknownFaceLabeler import UnknownFaceLabeler
from grouping.FaceGrouper import FaceGrouper
from grouping.SessionMemory import SessionMemory

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
)

video_capture = cv2.VideoCapture(iana_config.camera.CameraMode.NOTEBOOK_CAMERA)
video_capture.set(3, iana_config.camera.width)
video_capture.set(4, iana_config.camera.height)
interrupted = False

pos_frame = video_capture.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
while True:
    ret, frame = video_capture.read()
    if ret:
        pd.detect_person(frame, time.time())
    else:
        # The next frame is not ready, so we try to read it again
        video_capture.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, pos_frame - 1)

