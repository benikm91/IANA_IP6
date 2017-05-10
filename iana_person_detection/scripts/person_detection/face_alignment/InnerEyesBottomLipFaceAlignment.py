import openface

from person_detection.face_alignment.FaceAlignment import FaceAlignment


class InnerEyesBottomLipFaceAlignment(FaceAlignment):

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

    def align_faces(self, image, bounding_boxes):
        aligned_faces = []
        for box in bounding_boxes:
            aligned_faces.append(
                self.align.align(
                    self.img_dim,
                    image,
                    box,
                    landmarkIndices=openface.AlignDlib.INNER_EYES_AND_BOTTOM_LIP))
        return aligned_faces
