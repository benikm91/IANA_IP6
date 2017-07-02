#!/usr/bin/env python
import rospy
from controller.speech_controller import SpeechController

if __name__ == '__main__':
    try:
        rospy.init_node('iana_driver_random', anonymous=True)
        controller = SpeechController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
