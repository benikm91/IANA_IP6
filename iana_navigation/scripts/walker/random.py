import rospy
import geometry_msgs.msg
import std_msgs.msg

class RandomWalker(object):

    def __init__(self):
        super().__init__()
        self.collision_ahead = False
        self.interrupted = False
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', geometry_msgs.msg.Twist, queue_size=1)
        rospy.Subscriber('/collision_ahead', std_msgs.msg.Float32, self.collision_ahead_callback, queue_size=10)

    def start(self):
        self.interrupted = False
        while not self.interrupted:
            if self.collision_ahead:
                pass
            else:
                pass


    def stop(self):


    def collision_ahead_callback(self, msg):
        self.collision_ahead = msg.data < 0.8

