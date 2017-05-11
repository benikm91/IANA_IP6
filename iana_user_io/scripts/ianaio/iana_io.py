import rospy
from std_msgs.msg import Time
from geometry_msgs.msg import Pose
from iana_controller.msg import Explore, GoTo


class IanaTalker(object):

    def __init__(self):
        self.explore_pub = rospy.Publisher('/controller/user_command/explore', Explore, queue_size=10)
        self.goto_pub = rospy.Publisher('/controller/user_command/go_to', GoTo, queue_size=10)

    def start(self):
        pass

    def explore(self):
        time = Time()
        time.data = rospy.Time.now() + rospy.Duration(5*60)
        self.explore_pub.publish(time)

    def goto(self, x, y):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        self.goto_pub.publish(pose)


class IanaIO(object):

    def __init__(self, publisher):
        self.publisher = publisher

    def start(self):
        pass

    def request_name(self):
        pass
