#!/usr/bin/env python
import rospy

from controller.navigation_controller import NavigationController

if __name__ == '__main__':
    try:
        rospy.init_node('iana_navigation', anonymous=True)
        controller = NavigationController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
