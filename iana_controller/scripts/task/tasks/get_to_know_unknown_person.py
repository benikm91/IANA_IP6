import actionlib
import rospy
from iana_speech.msg import SayAction, SayGoal
from iana_user_io.msg import GetNameAction, GetNameGoal
from iana_person_data.srv import InsertNewPerson

from task.task import Task

from task.tasks.goodbye_unknown_person import GoodbyeUnknownPersonTask


class GetToKnowUnknownPersonTask(Task):

    def __init__(self, msg):
        super(GetToKnowUnknownPersonTask, self).__init__()
        self.face_vectors = msg.face_vectors
        self.preview_image = msg.preview_image
        self.get_name_action = actionlib.SimpleActionClient('/get_name', GetNameAction)
        if not self.get_name_action.wait_for_server(3):
            rospy.logerr("Can't find get name action.")
        self.say_action = actionlib.SimpleActionClient('/iana/speech/say', SayAction)
        if not self.say_action.wait_for_server(3):
            rospy.logerr("Can't find say action.")

        rospy.wait_for_service('insert_new_person')
        self.insert_new_person = rospy.ServiceProxy('/insert_new_person', InsertNewPerson)

    def _receive_name(self, state, result):
        name = result.name
        if name == -1:
            return

        rospy.loginfo('Insert new person with name "{0}"'.format(name))
        person = self.insert_new_person(name, self.face_vectors).person

        # check person result
        if person is None:
            rospy.logerr('Error while inserting new person')
        elif person.name != name:
            rospy.logerr('Inserted Person has wrong name. Name "{0}" given, got "{1}".'.format(name, person.name))

        self.terminated.set()

    @property
    def name(self):
        return "Get to know unknown Person"

    def update(self, elapsed):
        pass

    def on_start(self):
        rospy.loginfo('Ask and receive unknown persons name.')
        self.say_action.send_goal(SayGoal("Oh hello there. Please tell me who you are!"))
        self.get_name_action.send_goal(GetNameGoal(preview_image=self.preview_image), self._receive_name)

    def on_resume(self):
        self.terminated.set()

    def on_interrupt(self):
        self.terminated.set()

    def on_shutdown(self):
        self.get_name_action.cancel_goal()
        self.terminated.set()

    def interruptable_by(self, task):
        print (type(task), type(task) is GoodbyeUnknownPersonTask)
        return type(task) is GoodbyeUnknownPersonTask

