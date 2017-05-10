from person_detection.data_access.PersonDataAccess import PersonDataAccess


class Person(object):

    def __init__(self, id, name):
        self.id = id
        self.name = name


class PersonDataAccessInMemory(PersonDataAccess):

    def __init__(self):
        self.persons = list()
        self.next_id = 0

    def get(self, id):
        return self.persons[id]

    def get_all(self):
        return self.persons

    def insert(self, name):
        person_id = self.next_id
        self.persons.append(Person(person_id, name))
        self.next_id += 1
        return person_id
