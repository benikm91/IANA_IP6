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

from driver_explore_state import ExploreFrontiersState


class DriverExplore(object):

    def __init__(self):
        rospy.loginfo("Starting Driver Explore...")
        self.enabled = False
        self.occupancy_grid = None
        self.odometry = None
        self.state = ExploreFrontiersState(self)

        rospy.Subscriber("/iana/driver_explorer/enable", std_msgs.msg.Empty, self.enable)
        rospy.Subscriber("/iana/driver_explorer/disable", std_msgs.msg.Empty, self.disable)
        rospy.Subscriber("/map", nav_msgs.msg.OccupancyGrid, self.update_map)
        rospy.Subscriber("/odom", nav_msgs.msg.Odometry, self.update_odometry)

        rospy.loginfo("Init completed")

    def enable(self, msg):
        rospy.loginfo("Explore Driver enabled")
        self.enabled = True
        self.state.on_enable()

    def disable(self, msg):
        self.enabled = False
        self.state.on_disable()

    def update_map(self, occupancy_grid):
        self.occupancy_grid = occupancy_grid
        self.state.on_map_updated()

    def update_odometry(self, odometry):
        self.odometry = odometry
        self.state.on_odometry_updated()

    def update(self, delta_time):
        if self.enabled:
            self.state = self.state.update(delta_time)
