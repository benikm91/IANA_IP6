import actionlib
import rospy
from iana_speech.msg import SayAction, SayGoal
from iana_user_io.msg import GetNameAction, GetNameGoal
from iana_person_data.srv import InsertNewPerson

from taskv2.task import Task

from taskv2.tasks.goodbye_unknown_person import GoodbyeUnknownPersonTask


class GetToKnowUnknownPersonTask(Task):

    def __init__(self, msg):
        super(GetToKnowUnknownPersonTask, self).__init__()
        self.face_vectors = msg.face_vectors
        self.preview_image = msg.preview_image

    @property
    def name(self):
        return "Get to know unknown Person"

    def update(self, elapsed):
        pass

    def on_start(self):

        def greet_person():
            say_action.send_goal(SayGoal("Oh hello there. Please tell me who you are!"))

        def ask_and_receive_name():
            get_name_action.send_goal(GetNameGoal(preview_image=self.preview_image))
            get_name_action.wait_for_result()
            return get_name_action.get_result()

        rospy.loginfo('Setting up services and actions.')

        # setup -> init needed actions and services.
        get_name_action = actionlib.SimpleActionClient('/get_name', GetNameAction)
        get_name_action.wait_for_server()
        say_action = actionlib.SimpleActionClient('/iana/speech/say', SayAction)
        say_action.wait_for_server()
        rospy.wait_for_service('insert_new_person')
        insert_new_person = rospy.ServiceProxy('/insert_new_person', InsertNewPerson)

        rospy.loginfo('Greet unknown person.')
        greet_person()

        rospy.loginfo('Ask and receive unknown persons name.')
        name = ask_and_receive_name().name

        rospy.loginfo('Insert new person with name "{0}"'.format(name))
        person = insert_new_person(name, self.face_vectors).person

        # check person result
        if person is None:
            rospy.logerr('Error while inserting new person')
        elif person.name != name:
            rospy.logerr('Inserted Person has wrong name. Name "{0}" given, got "{1}".'.format(name, person.name))

        self.terminated.set()

    def on_resume(self):
        self.terminated.set()

    def on_interrupt(self):
        self.terminated.set()

    def on_shutdown(self):
        self.terminated.set()

    def interruptable_by(self, task):
        print (type(task), type(task) is GoodbyeUnknownPersonTask)
        return type(task) is GoodbyeUnknownPersonTask

