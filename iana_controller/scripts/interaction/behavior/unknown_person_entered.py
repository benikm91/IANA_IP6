import os
import cv2

from iana_person_detection.msg import KnownPersonEntered, KnownPersonLeft, UnknownPersonEntered, UnknownPersonLeft

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
    def unknown_person_entered(self, iana, event):
        message = "Hello Unknown?"
        event.preview_image.encoding = "bgr8"
        frame = bridge.imgmsg_to_cv2(event.preview_image, desired_encoding="bgr8")
        cv2.imshow("Display window", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        os.system("say %(message)s" % locals())