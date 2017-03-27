
class FaceClassifier(object):

    def train(self, face_data):
        """
        Trains the classifier with the given data
        :param face_data: List of tuples containing person_id (label) and feature vector (shape: m x 1)
        :type face_data: list[(int, np.ndarray)]
        :return: void
        """
        pass

    def update(self, person_id, feature_vectors):
        """
        Updates the classifier with the given data
        This might cause the classifier to retrain itself (depending on implementation)
        :param person_id: The person_id (label)
        :type person_id: int
        :param feature_vectors: feature vector (shape: m x 1)
        :type feature_vectors: [np.ndarray]
        :return: void
        """
        pass

    def predict(self, face_embedding):
        """
        Predicts the label of the given feature_vector using the pre-trained classification model
        :param face_embedding:
        :return: A tuple containing (person_id, confidence)
        :rtype: (int, float)
        """
        pass
