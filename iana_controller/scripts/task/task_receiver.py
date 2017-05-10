import rospy
from iana_controller.msg import Explore, GoTo

from task.task_system import TaskSystem


class TaskReceiver(object):

    """
    :type task_system: TaskSystem
    """

    def __init__(self, task_system):
        super(TaskReceiver, self).__init__()
        self.task_system = task_system
        rospy.Subscriber('/controller/user_command/explore', Explore, self.explore, queue_size=10)
        rospy.Subscriber('/controller/user_command/go_to', GoTo, self.explore, queue_size=10)

    def explore(self, msg):
        self.task_system.submit(msg.until)

    def go_to(self, msg):
        self.task_system.submit(msg.goal)

