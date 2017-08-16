import rospy
from std_msgs.msg import Time
from geometry_msgs.msg import PoseStamped
from iana_controller.msg import Explore, ExploreRandom, GoTo


class IanaTalker(object):

    def __init__(self):
        self.explore_pub = rospy.Publisher('/iana/user_command/explore', Explore, queue_size=10)
        self.explore_random_pub = rospy.Publisher('/iana/user_command/explore_random', ExploreRandom, queue_size=10)
        self.goto_pub = rospy.Publisher('/iana/user_command/go_to', GoTo, queue_size=10)

    def start(self):
        pass

    def explore(self, seconds):
        time = Time()
        time.data = rospy.Time.now() + rospy.Duration(int(seconds))
        self.explore_pub.publish(time)

    def explore_random(self, seconds):
        time = Time()
        time.data = rospy.Time.now() + rospy.Duration(int(seconds))
        self.explore_random_pub.publish(time)

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

    def request_name(self, preview_image):
        pass

    def refresh_map(self, resolution, origin, width, height, origin_x, origin_y, map):
        pass