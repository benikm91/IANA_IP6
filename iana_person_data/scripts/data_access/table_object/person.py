from data_access.table_object.base import Base
from sqlalchemy import Column
from sqlalchemy import Integer
from sqlalchemy import Sequence
from sqlalchemy import String


class Person(Base):
    __tablename__ = 'person'
    id = Column(Integer, Sequence('id'), primary_key=True, autoincrement=True)
    name = Column(String(255))

