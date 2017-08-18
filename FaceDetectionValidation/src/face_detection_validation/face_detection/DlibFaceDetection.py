import openface

from face_detection_validation.face_detection.FaceDetection import FaceDetection


class DlibFaceDetection(FaceDetection):

    def __init__(self, predictor_path):
        """
        :param predictor_path:
        :type predictor_path: str
        """
        super(FaceDetection, self).__init__()
        self.predictor_path = predictor_path
        self.align = openface.AlignDlib(predictor_path)

    def detect_faces(self, image):
        return sorted(self.align.getAllFaceBoundingBoxes(image), key=lambda rec: rec.width() * rec.height(), reverse=True)
