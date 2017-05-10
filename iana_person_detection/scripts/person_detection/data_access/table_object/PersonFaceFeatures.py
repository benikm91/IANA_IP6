from sqlalchemy import Column
from sqlalchemy import Integer
from sqlalchemy import LargeBinary
from person_detection.data_access.table_object.Base import Base


class PersonFaceFeatures(Base):
    __tablename__ = 'person_face_features'
    person_id = Column(Integer, primary_key=True, autoincrement=False)
    face_features = Column(LargeBinary)
