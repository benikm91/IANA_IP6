from classifier.FaceClassifier import FaceClassifier


class FaceLabeler(object):
    """
    :type classifier: FaceClassifier
    """

    def __init__(self, classifier):
        """
        :param classifier:
        :type classifier: FaceClassifier
        """
        self.classifier = classifier

    def label(self, face_vectors):
        """
        Labels a list of face vectors
        :param face_vectors: List of face_vectors
        :type face_vectors: np.ndarray
        :return: List of labeled face_vectors
        :rtype: list[(person_id, confidence, face_vector)]
        """
        labeled_faces = []
        for face_vector in face_vectors:
            person_id, confidence = self.classifier.predict(face_vector)
            labeled_faces.append((person_id, confidence, face_vector))
        return labeled_faces
