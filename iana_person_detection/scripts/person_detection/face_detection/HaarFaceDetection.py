import cv2
import openface

from person_detection.face_detection.FaceDetection import FaceDetection


class HaarFaceDetector(FaceDetection):

    def __init__(self, model):
        self.face_cascade = cv2.CascadeClassifier(model)

    def detect_faces(self, image):
        bounding_boxes = self.face_cascade.detectMultiScale(image, 1.3, 5)
        return sorted(bounding_boxes, key=lambda rec: rec[2] * rec[3], reverse=True)