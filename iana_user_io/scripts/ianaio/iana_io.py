import rospy
from std_msgs.msg import String


class IanaTalker(object):

    def __init__(self):
        self.explore_pub = rospy.Publisher('explorer', String, queue_size=10)
        self.goto_pub = rospy.Publisher('goto', String, queue_size=10)

    def start(self):
        pass

    def explore(self):
        self.explore_pub.publish("EXPLORE")

    def goto(self, x, y):
        self.goto_pub.publish("GOTO")


class IanaIO(object):

    def __init__(self, publisher):
        self.publisher = publisher

    def start(self):
        pass

    def request_name(self):
        pass
