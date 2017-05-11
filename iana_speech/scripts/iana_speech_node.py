#!/usr/bin/env python
import rospy
from controller.speech_controller import SpeechController

if __name__ == '__main__':
    try:
        rospy.init_node('iana_speech', anonymous=True)
        controller = SpeechController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
