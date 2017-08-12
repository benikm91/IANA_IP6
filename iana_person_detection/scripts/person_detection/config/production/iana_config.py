import pkg_resources

from person_detection.config.Config import Config

camera = Config(
    capture_device=Config.CameraMode.NOTEBOOK_CAMERA,
    width=800,
    height=600
)
face_detection = Config(
    predictor_path=pkg_resources.resource_filename('res', 'models/dlib/shape_predictor_68_face_landmarks.dat'),
    img_dim=96,
)
face_embedder = Config(
    network_model=pkg_resources.resource_filename('res', 'models/openface/nn4.small2.v1.t7'),
    img_dim=96,
    cuda=True
)
clusterer = Config(
    threshold_same = 0.75
)
filter = Config(
    threshold_known = 0.75,
    threshold_unknown = 0.6
)
grouper = Config(
    known_threshold = 2,
    unknown_threshold = 5,
    memory_span_known = 20,
    memory_span_unknown = 20
)
session_memory = Config(
    memory_span_known = 20,
    memory_span_unknown = 20
)
iana = Config(
    memory_span = 60 * 60 * 15
)
recorder = Config(
    threshold_same = 0.75,
    min_faces = 3
)
debug = Config (
    logging_strategy = Config.LoggingStrategy.NO_LOGGING
)
