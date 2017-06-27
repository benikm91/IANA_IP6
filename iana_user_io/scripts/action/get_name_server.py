import rospy
import actionlib
from iana_user_io.msg import GetNameAction, GetNameGoal, GetNameFeedback, GetNameResult

from cv_bridge import CvBridge

bridge = CvBridge()


class GetNameActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = GetNameFeedback()
    _result = GetNameResult()

    def __init__(self, name, io):
        """
        :param name: 
        :param io:
        :type io: iana_io.iana_io.IanaIO
        """
        self._action_name = name
        self.io = io
        self._as = actionlib.SimpleActionServer(self._action_name, GetNameAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        goal.preview_image.encoding = "bgr8"
        self._result.name = self.io.request_name(bridge.imgmsg_to_cv2(goal.preview_image, desired_encoding="bgr8"))
        self._as.set_succeeded(self._result)
