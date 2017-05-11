import rospy

from iana_controller.msg import Explore, GoTo
from tasks.explore import ExploreTask
from tasks.goto import GoToTask
from std_msgs.msg import Time

class TaskReceiver(object):

    def __init__(self, task_list):
        super(TaskReceiver, self).__init__()
        self.task_list = task_list
        rospy.Subscriber('/controller/user_command/explore', Explore, self.explore, queue_size=10)
        rospy.Subscriber('/controller/user_command/go_to', GoTo, self.explore, queue_size=10)

    def explore(self, msg):
        rospy.loginfo('Submit ExploreTask to task_list')
        now = rospy.get_rostime()
        now.secs += 10
        self.task_list.submit(ExploreTask(Time(data=now)))

    def go_to(self, msg):
        self.task_list.submit(GoToTask(msg))

