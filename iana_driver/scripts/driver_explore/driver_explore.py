import math
import random

import numpy as np
import rospy
import actionlib

import std_msgs.msg
import move_base_msgs.msg
import geometry_msgs.msg
import actionlib_msgs.msg
import nav_msgs.msg

from driver_explore.driver_explore_state import ExploreFrontiersState


class DriverExplore(object):

    def __init__(self):
        self.enabled = False
        self.occupancy_grid = None
        self.state = ExploreFrontiersState(self)

        rospy.Subscriber("/iana/driver_explorer/enable", std_msgs.msg.Empty, self.enable)
        rospy.Subscriber("/iana/driver_explorer/disable", std_msgs.msg.Empty, self.disable)
        rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, self.update_map)

        self.move_base_action = actionlib.SimpleActionClient('/move_base', move_base_msgs.msg.MoveBaseAction)
        if not self.move_base_action.wait_for_server(rospy.Duration(30)):
            rospy.logerr('Failed to connect to /move_base action')

    def enable(self, msg):
        self.enabled = True
        self.state.on_enable()

    def disable(self, msg):
        self.enabled = False
        self.state.on_disable()

    def update_map(self, occupancy_grid):
        self.occupancy_grid = occupancy_grid
        self.state.on_map_updated()

    def update(self, delta_time):
        if self.enabled:
            self.state.update(delta_time)
