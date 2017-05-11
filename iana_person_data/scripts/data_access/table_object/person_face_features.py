from data_access.table_object.base import Base
from sqlalchemy import Column
from sqlalchemy import Integer
from sqlalchemy import LargeBinary


class PersonFaceFeatures(Base):
    __tablename__ = 'person_face_features'
    person_id = Column(Integer, primary_key=True, autoincrement=False)
    face_features = Column(LargeBinary)
