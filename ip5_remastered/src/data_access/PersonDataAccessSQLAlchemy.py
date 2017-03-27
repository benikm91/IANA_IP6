from data_access.PersonDataAccess import PersonDataAccess
from data_access.table_object.Person import Person

class PersonDataAccessSQLAlchemy(PersonDataAccess):

    @staticmethod
    def __to_person(person_data):
        return Person(id=person_data.id, name=person_data.name)

    def __init__(self, engine, session_maker):
        self.engine = engine
        self.session_maker = session_maker

    def get(self, id):
        session = self.session_maker()
        person_data = session.query(Person).filter(Person.id == id).first()
        return self.__to_person(person_data)

    def get_all(self):
        session = self.session_maker()
        persons_data = session.query(Person).all()
        return map(self.__to_person, persons_data)

    def insert(self, name):
        session = self.session_maker()
        new_person = Person(name=name)
        session.add(new_person)
        session.commit()
        return new_person.id