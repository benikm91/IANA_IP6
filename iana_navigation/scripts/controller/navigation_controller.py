import rospy
import actionlib

import iana_navigation.msg
import move_base_msgs.msg

from iana_navigation.action import Explore, GoTo, Idle


class NavigationController(object):

    def __init__(self):
        rospy.init_node('iana_navigation', anonymous=True)

        self.explore_action = actionlib.SimpleActionServer('/iana/navigation/explore', Explore, execute_cb=self.explore, auto_start=True)
        self.go_to_action = actionlib.SimpleActionServer('/iana/navigation/go_to', GoTo, execute_cb=self.go_to, auto_start=True)

        self.move_base_action = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        self.move_base_action.wait_for_server()

    def run(self):
        rospy.spin()

    def explore(self, goal):
        pass

    def go_to(self, goal):
        base_goal = move_base_msgs.msg.MoveBaseActionGoal(target_pose=goal.target_pose)

        self.move_base_action.send_goal(base_goal)

        preempted = False
        while not self.move_base_action.wait_for_result(0.1) and not preempted:
            if self.go_to_action.is_preempt_requested():
                self.move_base_action.cancel_goal()
                self.go_to_action.set_preempted()
                preempted = True

        base_result = self.move_base_action.get_result()
        result = iana_navigation.msg.GoToResult(base_position=base_result.base_position)

        if not preempted:
            self.go_to_action.set_succeeded(result)
