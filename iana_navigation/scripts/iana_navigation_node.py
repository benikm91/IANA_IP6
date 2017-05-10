#!/usr/bin/env python
import rospy

from scripts.controller.navigation_controller import NavigationController

if __name__ == '__main__':
    try:
        controller = NavigationController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
