import numpy as np
from collections import defaultdict
from time import sleep

from person_detection.classifier.FaceClassifier import FaceClassifier


class DistanceToAverageFaceClassifier(FaceClassifier):
    """
    Face embedding classifier using a distance function for classification
    :type faces: dict[int, Face]
    """

    def __init__(self, distance, confidence, average=np.average):
        self.faces = defaultdict(lambda: None)
        self.distance = distance
        self.confidence = confidence
        self.average = average

    def train(self, persons):
        faces = defaultdict(lambda: None)
        for person in persons:
            faces[person.person_id] = map(lambda x: x.face, person.face_vectors)
        for person_id, feature_vectors in faces.iteritems():
            self.__insert(person_id, feature_vectors)

    def update(self, person_id, feature_vectors):
        known_face = self.faces[person_id]
        if known_face is None:
            self.__insert(person_id, feature_vectors)
        else:
            face_embedding_count = len(feature_vectors)
            face_embedding_sum = np.sum(np.array(feature_vectors), axis=0)
            feature_vectors_count = known_face.feature_vectors_count + face_embedding_count #+ 1
            known_face.centroid = np.true_divide(
                (known_face.centroid * known_face.feature_vectors_count) + face_embedding_sum, feature_vectors_count)
            known_face.feature_vectors_count = feature_vectors_count

    def predict(self, face_embedding):
        closest_face, min_dist = self.__get_closest(face_embedding)
        confidence = self.confidence(min_dist)
        return closest_face, confidence

    def __insert(self, person_id, feature_vectors):
        centroid = self.average(np.array(feature_vectors), axis=0)
        self.faces[person_id] = CentroidFace(person_id, centroid, len(feature_vectors))

    def __get_closest(self, face_embedding):
        min_dist = float("inf")
        closest_face = -1
        for person_id, face in self.faces.iteritems():
            dist = self.distance(face_embedding, face.centroid)
            if dist < min_dist:
                min_dist = dist
                closest_face = face.person_id
        return closest_face, min_dist


class CentroidFace(object):
    def __init__(self, person_id, centroid, feature_vectors_count):
        self.person_id = person_id
        self.centroid = centroid
        self.feature_vectors_count = feature_vectors_count
