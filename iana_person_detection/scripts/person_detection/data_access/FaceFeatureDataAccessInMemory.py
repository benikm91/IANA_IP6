from collections import defaultdict
from person_detection.data_access.FaceFeatureDataAccess import FaceFeatureDataAccess


class FaceFeatureDataAccessInMemory(FaceFeatureDataAccess):

    def __init__(self):
        self.face_features = defaultdict(list)
        self.next_id = 0

    def get_all(self):
        """
        :return:
        :rtype: list[(int, np.ndarray)]
        """
        return self.face_features.iteritems()

    def insert(self, person_id, features):
        """
        :param person_id:
        :type person_id: int
        :param features:
        :type features: numpy.ndarray
        :return:
        """
        if not features is list:
            features = list(features)
        self.face_features[person_id] = features
