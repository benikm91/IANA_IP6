from data_service.person_service import PersonService


class PersonServiceImpl(PersonService):

    def __init__(self, person_dao, face_feature_dao):
        """
        :param person_dao:
        :type person_dao: data_access.person_dao.PersonDao
        :param face_feature_dao: 
        :type face_feature_dao: data_access.face_feature_dao.FaceFeatureDao
        """
        self.person_dao = person_dao
        self.face_feature_dao = face_feature_dao

    def _join_face_features(self, person):
        person.face_features = self.face_feature_dao.get_all_for_person(person.id)
        return person

    def get_all_with_face_vectors(self):
        return map(self._join_face_features, self.person_dao.get_all())

    def get_with_face_vectors(self, person_id):
        person = self.person_dao.get(person_id)
        return self._join_face_features(person)

    def insert(self, label, face_vectors):
        person_id = self.person_dao.insert(label)
        self.face_feature_dao.insert(person_id, face_vectors)
        return person_id
