from iana_person_data.srv import GetAllPersonsResponse, InsertNewPersonResponse
from iana_person_data.msg import Person, FaceVector


class PersonData(object):

    def __init__(self, person_service):
        """
        :param person_service: 
        :type person_service: data_service.person_service.PersonService
        """
        self.person_service = person_service

    def person_to_msg(self, person):
        person_msg = Person()
        person_msg.person_id = person.id
        person_msg.name = person.name.encode('ascii', 'ignore')
        person_msg.face_vectors = map(FaceVector, person.face_features)
        return person_msg

    def handle_get_all(self, request):

        persons_msg = map(self.person_to_msg, self.person_service.get_all_with_face_vectors())
        return GetAllPersonsResponse(persons_msg)

    def handle_insert(self, request):
        person_id = self.person_service.insert(request.name, map(lambda x: x.face, request.face_vectors))
        return self.person_to_msg(self.person_service.get_with_face_vectors(person_id))

    def handle_update_features(self, request):
        self.person_service.update_features(request.person_id, map(lambda x: x.face, request.face_vectors))
        return self.person_to_msg(self.person_service.get_with_face_vectors(request.person_id))
