

class FaceFeatureDao(object):

    def get_all(self):
        """
        :return:
        :rtype: list[(int, np.ndarray)]
        """
        raise NotImplementedError

    def get_all_for_person(self, person_id):
        raise NotImplementedError

    def insert(self, person_id, features):
        """
        :param person_id:
        :type person_id: int
        :param features:
        :type features: numpy.ndarray
        :return:
        """
        raise NotImplementedError
