#!/usr/bin/env python

import rospy
from controller.iana_controller import IanaController

if __name__ == '__main__':

    try:
        rospy.init_node('iana_controller', anonymous=True)
        controller = IanaController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
