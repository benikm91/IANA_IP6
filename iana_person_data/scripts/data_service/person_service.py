class PersonService(object):

    def get_all_with_face_vectors(self):
        """
        :return:
        :rtype: list[Person]
        """
        raise NotImplementedError

    def get_with_face_vectors(self, person_id):
        raise NotImplementedError

    def insert(self, label, face_vectors):
        """
        :param label:
        :type label: string
        :return: id of inserted person
        :rtype int
        """
        raise NotImplementedError
