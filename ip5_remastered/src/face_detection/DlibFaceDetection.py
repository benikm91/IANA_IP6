import openface

from face_detection.FaceDetection import FaceDetection


class DlibFaceDetector(FaceDetection):

    def __init__(self, predictor_path, img_dim):
        """
        :param predictor_path:
        :type predictor_path: str
        :param img_dim:
        :type img_dim: int
        """
        self.predictor_path = predictor_path
        self.img_dim = img_dim
        self.align = openface.AlignDlib(predictor_path)

    def detect_faces(self, image):
        return sorted(self.align.getAllFaceBoundingBoxes(image), key=lambda rec: rec.width() * rec.height(), reverse=True)
