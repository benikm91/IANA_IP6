class PersonCache(object):

    def __init__(self):
        self.persons = dict()

    def insert(self, person):
        self.persons[person.person_id] = person

    def __getitem__(self, id):
        return self.persons[id]