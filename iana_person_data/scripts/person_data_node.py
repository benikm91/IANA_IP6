#!/usr/bin/env python

import rospy

rospy.init_node('person_data_node', anonymous=True)

from data_access.face_feature_dao_sqlalchemy import FaceFeatureDaoSQLAlchemy
from data_access.person_dao_sqlalchemy import PersonDaoSQLAlchemy
from data_access.table_object.base import Base
from data_service.person_service_impl import PersonServiceImpl
from iana_person_data.msg import Person
from person_data.person_data import PersonData
from data_access.table_object.sqllite import session_maker, engine_instance

from iana_person_data.srv import GetAllPersons, InsertNewPerson
from std_msgs.msg import Int64

if __name__ == '__main__':

    publisher = rospy.Publisher('new_person', Person, queue_size=10)

    Base.metadata.create_all(engine_instance)

    person_service = PersonServiceImpl(
        PersonDaoSQLAlchemy(engine_instance, session_maker),
        FaceFeatureDaoSQLAlchemy(engine_instance, session_maker)
    )

    person_data = PersonData(person_service)

    try:

        def handle_insert_and_notify(request):
            person = person_data.handle_insert(request)
            publisher.publish(person)
            return person

        get_all_persons = rospy.Service('get_all_persons', GetAllPersons, person_data.handle_get_all)
        insert_new_person = rospy.Service('insert_new_person', InsertNewPerson, handle_insert_and_notify)

        rospy.spin()
    except rospy.ROSInterruptException: pass