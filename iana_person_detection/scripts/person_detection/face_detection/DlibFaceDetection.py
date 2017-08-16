import openface

from person_detection.face_detection.FaceDetection import FaceDetection


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

    def dlib_rectangle_to_bounding_box(self, rect):
        """
        :param rect:
        :type rect: rectangle
        :return: (x, y, width, height)
        """
        return rect.left(), rect.top(), rect.width(), rect.height()

    def detect_faces(self, image):
        bounding_boxes = map(self.dlib_rectangle_to_bounding_box, self.align.getAllFaceBoundingBoxes(image))
        return sorted(bounding_boxes, key=lambda rec: rec[2] * rec[3], reverse=True)
