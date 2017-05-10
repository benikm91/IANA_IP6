from task.task import Task

import rospy
from std_msgs.msg import String


class ExploreTask(Task):

    def __init__(self, until=-1):
        super(ExploreTask, self).__init__()
        self.until = until
        self.navigation_state_publisher = rospy.Publisher('/controller/navigation/set_state', String, queue_size=10)
        rospy.Subscriber('/controller/navigation/explored', String, self.finished, queue_size=10)

    def run(self):
        self.navigation_state_publisher.publish('explore', self.until)

    def resume(self):
        self.run()
