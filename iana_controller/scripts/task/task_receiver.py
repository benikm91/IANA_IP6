import rospy

from iana_controller.msg import Explore, ExploreRandom, GoTo
from iana_person_detection.msg import UnknownPersonEntered, KnownPersonEntered, UnknownPersonLeft, KnownPersonLeft
from tasks.explore_random import ExploreRandomTask
from tasks.goodbye_unknown_person import GoodbyeUnknownPersonTask
from tasks.greet_known_person import GreetKnownPersonTask
from tasks.get_to_know_unknown_person import GetToKnowUnknownPersonTask
from tasks.explore import ExploreTask
from tasks.goto import GoToTask
from tasks.goodbye_known_person import GoodbyeKnownPersonTask


class TaskReceiver(object):

    def __init__(self, task_list):
        super(TaskReceiver, self).__init__()
        self.task_list = task_list
        rospy.Subscriber('/iana/user_command/explore', Explore, self.explore, queue_size=10)
        rospy.Subscriber('/iana/user_command/explore_random', ExploreRandom, self.explore_random, queue_size=10)
        rospy.Subscriber('/iana/user_command/go_to', GoTo, self.go_to, queue_size=10)
        rospy.Subscriber('/iana/person_detection/unknown/entered', UnknownPersonEntered, self.unknown_person_entered, queue_size=10)
        rospy.Subscriber('/iana/person_detection/known/entered', KnownPersonEntered, self.known_person_entered, queue_size=10)
        rospy.Subscriber('/iana/person_detection/unknown/left', UnknownPersonLeft, self.unknown_person_left, queue_size=10)
        rospy.Subscriber('/iana/person_detection/known/left', KnownPersonLeft, self.known_person_left, queue_size=10)

    def explore(self, msg):
        self.task_list.submit(ExploreTask(msg))

    def explore_random(self, msg):
        self.task_list.submit(ExploreRandomTask(msg))

    def go_to(self, msg):
        self.task_list.submit(GoToTask(msg))

    def unknown_person_entered(self, msg):
        self.task_list.submit(GetToKnowUnknownPersonTask(msg))

    def known_person_entered(self, msg):
        self.task_list.submit(GreetKnownPersonTask(msg))

    def unknown_person_left(self, msg):
        self.task_list.submit(GoodbyeUnknownPersonTask(msg))

    def known_person_left(self, msg):
        self.task_list.submit(GoodbyeKnownPersonTask(msg))
