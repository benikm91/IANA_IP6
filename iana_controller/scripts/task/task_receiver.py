import rospy

from iana_controller.msg import Explore, GoTo
from task.tasks.explore import ExploreTask
from task.tasks.goto import GoToTask


class TaskReceiver(object):

    def __init__(self, task_list):
        super(TaskReceiver, self).__init__()
        self.task_list = task_list
        rospy.Subscriber('/controller/user_command/explore', Explore, self.explore, queue_size=10)
        rospy.Subscriber('/controller/user_command/go_to', GoTo, self.explore, queue_size=10)

    def explore(self, msg):
        self.task_list.submit(ExploreTask(msg))

    def go_to(self, msg):
        self.task_list.submit(GoToTask(msg))

