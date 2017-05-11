#!/usr/bin/env python
import rospy

from mapping.mapping_controller import MappingController

if __name__ == "__main__":

    try:
        rospy.init_node('iana_orientation', anonymous=True)
        controller = MappingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
