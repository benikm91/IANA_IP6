from clusterer.FaceClusterer import FaceClusterer


class UnknownFaceLabeler(object):
    """
    :type clusterer: FaceClusterer
    """

    def __init__(self, clusterer):
        """
        :param clusterer:
        :type clusterer: FaceClusterer
        """
        self.clusterer = clusterer

    def label(self, face_vector):
        """
        Labels a list of face vectors
        :param face_vector:
        :type face_vector: np.ndarray
        :return: label
        :rtype: int
        """
        return self.clusterer.cluster(face_vector)
