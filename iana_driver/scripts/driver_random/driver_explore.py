import math
import random

import numpy as np
import rospy
import actionlib

import std_msgs.msg
import move_base_msgs.msg
import geometry_msgs.msg
import actionlib_msgs.msg

class DriverExplore(object):

    def __init__(self):
        self.enabled = False
        self.base_goal = None

        # self.collision_ahead_threshold = rospy.get_param("collision_ahead_threshold", 0.8)

        rospy.Subscriber("/iana/driver_explorer/enable", std_msgs.msg.Empty, self.enable)
        rospy.Subscriber("/iana/driver_explorer/disable", std_msgs.msg.Empty, self.disable)

        self.move_base_action = actionlib.SimpleActionClient('/move_base', move_base_msgs.msg.MoveBaseAction)
        if not self.move_base_action.wait_for_server(rospy.Duration(30)):
            rospy.logerr('Failed to connect to /move_base action')

    def enable(self, msg):
        self.enabled = True

    def disable(self, msg):
        if self.base_goal is not None:
            self.base_goal.cancel_goal()
        self.enabled = False

    def next_target_pose(self):
        target_pose = geometry_msgs.msg.PoseStamped()
        return target_pose

    def update(self, delta_time):
        if self.enabled:
            if self.base_goal is None:
                target_pose = self.next_target_pose()
                self.base_goal = move_base_msgs.msg.MoveBaseGoal(target_pose=target_pose)

            if self.base_goal.wait_for_result():
                self.base_goal = None
