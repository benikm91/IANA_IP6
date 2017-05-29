import rospy
from std_msgs.msg import Time
from geometry_msgs.msg import PoseStamped
from iana_controller.msg import Explore, GoTo


class IanaTalker(object):

    def __init__(self):
        self.explore_pub = rospy.Publisher('/iana/user_command/explore', Explore, queue_size=10)
        self.goto_pub = rospy.Publisher('/iana/user_command/go_to', GoTo, queue_size=10)

    def start(self):
        pass

    def explore(self):
        time = Time()
        time.data = rospy.Time.now() + rospy.Duration(5*60)
        self.explore_pub.publish(time)

    def goto(self, x, y, qx, qy, qz, qw):
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.x = float(qx)
        target_pose.pose.orientation.y = float(qy)
        target_pose.pose.orientation.z = float(qz)
        target_pose.pose.orientation.w = float(qw)
        self.goto_pub.publish(target_pose)


class IanaIO(object):

    def __init__(self, publisher):
        self.publisher = publisher

    def start(self):
        pass

    def request_name(self):
        pass

    def refresh_map(self, width, height, map):
        pass