import pickle

from data_access.face_feature_dao import FaceFeatureDao
from data_access.table_object.person_face_features import PersonFaceFeatures


class FaceFeatureDaoSQLAlchemy(FaceFeatureDao):

    @staticmethod
    def __to_face_feature(face_feature_data):
        return pickle.loads(face_feature_data.face_features)

    def __init__(self, engine, session_maker):
        self.engine = engine
        self.session_maker = session_maker

    def get_all_for_person(self, person_id):
        session = self.session_maker()
        face_feature_data = session.query(PersonFaceFeatures).filter_by(person_id=person_id).first()
        return self.__to_face_feature(face_feature_data)

    def get_all(self):
        session = self.session_maker()
        face_feature_data = session.query(PersonFaceFeatures).all()
        return map(self.__to_face_feature, face_feature_data)

    def insert(self, person_id, face_features):
        if not face_features is list:
            face_features = list(face_features)
        session = self.session_maker()
        to_update = session.query(PersonFaceFeatures).filter_by(person_id=person_id).first()
        if to_update == None:
            data = pickle.dumps(face_features)
            new_feature = PersonFaceFeatures(person_id=person_id, face_features=data)
            session.add(new_feature)
        else:
            data = pickle.loads(to_update.face_features)
            to_update.face_features = pickle.dumps(data + face_features)
        session.commit()

    def update(self, person_id, face_features):
        if not face_features is list:
            face_features = list(face_features)
        session = self.session_maker()
        to_update = session.query(PersonFaceFeatures).filter_by(person_id=person_id).first()
        if to_update is None:
            return False
        else:
            to_update.face_features = pickle.dumps(face_features)
        session.commit()
        return True

