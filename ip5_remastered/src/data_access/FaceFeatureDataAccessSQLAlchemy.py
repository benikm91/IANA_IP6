import pickle

from data_access.table_object.PersonFaceFeatures import PersonFaceFeatures
from data_access.FaceFeatureDataAccess import FaceFeatureDataAccess


class FaceFeatureDataAccessSQLAlchemy(FaceFeatureDataAccess):

    @staticmethod
    def __to_face_feature(face_feature_data):
        return face_feature_data.person_id, pickle.loads(face_feature_data.face_features)

    def __init__(self, engine, session_maker):
        self.engine = engine
        self.session_maker = session_maker

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

