from scripts.task.task import Task

import rospy
from std_msgs.msg import String


class GoToTask(Task):

    def __init__(self, goal):
        super(GoToTask, self).__init__()
        self.goal = goal
        self.navigation_state_publisher = rospy.Publisher('/iana/navigation/set_state', String, queue_size=10)
        rospy.Subscriber('/iana/navigation/goal_reached', String, self.finished, queue_size=10)

    def run(self):
        self.navigation_state_publisher.publish('goto', self.goal)

    def resume(self):
        self.run()
