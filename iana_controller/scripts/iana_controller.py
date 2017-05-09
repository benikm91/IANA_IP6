#!/usr/bin/env python
import rospy

from iana_controller.msg import NavigationGoal, Explore
from scripts.states.mapping import OffMappingState
from scripts.states.navigation import IdleNavigationState


class IanaController(object):

    def __init__(self):
        self.navigation_state = IdleNavigationState()
        self.mapping_state = OffMappingState()
        self.mapping_state = OffMappingState()
        rospy.init_node('iana_controller', anonymous=True)
        rospy.Subscriber("/iana/controller_command/explore", Explore, self.explore, queue_size=10)
        rospy.Subscriber("/iana/controller_command/go_to", NavigationGoal, self.go_to, queue_size=10)

    def run(self):
        rospy.spin()

    def explore(self, epxlore):
        pass

    def go_to(self, go_to):
        pass


if __name__ == '__main__':
    try:
        controller = IanaController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
