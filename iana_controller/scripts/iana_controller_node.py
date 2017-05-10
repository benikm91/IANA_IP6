#!/usr/bin/env python

import rospy
from controller.iana_controller import IanaController

if __name__ == '__main__':

    try:
        controller = IanaController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
