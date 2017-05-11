import os

import rospy
import actionlib
from iana_person_data.srv import InsertNewPerson
from iana_user_io.msg import GetNameAction, GetNameGoal
import cv2

from iana_person_detection.msg import UnknownPersonEntered

from cv_bridge import CvBridge

bridge = CvBridge()

class UnknownPersonEnteredBehavior(object):
    def unknown_person_entered(self, iana, msg):
        """
        :param iana
        :type iana : Iana
        :param event:
        :type event UnknownPersonEntered
        :return:
        """
        pass


class GreetUnknownPerson(UnknownPersonEnteredBehavior):
    def unknown_person_entered(self, iana, msg):
        message = "Hello Unknown?"
        msg.preview_image.encoding = "bgr8"
        frame = bridge.imgmsg_to_cv2(msg.preview_image, desired_encoding="bgr8")
        cv2.imshow("Display window", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        os.system("say %(message)s" % locals())


class RegisterUnknownPerson(UnknownPersonEnteredBehavior):

    def __init__(self):
        self.insert_new_person_proxy = rospy.ServiceProxy('insert_new_person', InsertNewPerson)

    def unknown_person_entered(self, iana, msg):
        message = "Oh Hello there please tell me who you are!"
        os.system("say %(message)s" % locals())
        client = actionlib.SimpleActionClient('get_name', GetNameAction)
        client.wait_for_server()
        goal = GetNameGoal(msg.preview_image)
        client.send_goal(goal)
        client.wait_for_result()

        name = client.get_result().name

        self.insert_new_person_proxy(name, msg.face_vectors)

        print "Hallo", name