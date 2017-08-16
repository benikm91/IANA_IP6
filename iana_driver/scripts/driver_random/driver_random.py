import math
import random

import numpy as np
import rospy
import std_msgs.msg
import geometry_msgs.msg

from common.sensor_state import SensorState
from driver_random_state import DriverIdleState, DriverForwardState


class DriverRandom(object):

    def  __init__(self):
        rospy.loginfo("Starting random driver...")

        self.min_linear_velocity = rospy.get_param("min_linear_velocity", 0.05)
        self.medium_linear_velocity = rospy.get_param("medium_linear_velocity", 0.125)
        self.max_linear_velocity = rospy.get_param("max_linear_velocity", 0.25)
        self.min_angular_velocity = rospy.get_param("min_angular_velocity", 0.1)
        self.max_angular_velocity = rospy.get_param("max_angular_velocity", 1)
        self.linear_acceleration = rospy.get_param("linear_acceleration", 0.2)
        self.angular_acceleration = rospy.get_param("angular_acceleration", 0.2)
        self.collision_info_threshold = rospy.get_param("collision_info_threshold", 2)
        self.collision_warn_threshold = rospy.get_param("collision_warn_threshold", 0.8)
        self.collision_ahead_threshold = rospy.get_param("collision_ahead_threshold", 0.2)

        self.command_velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', geometry_msgs.msg.Twist, queue_size=10)

        rospy.Subscriber("/iana/driver_random/enable", std_msgs.msg.Empty, self.enable)
        rospy.Subscriber("/iana/driver_random/disable", std_msgs.msg.Empty, self.disable)
        rospy.Subscriber("/iana/collision_detector/collision_ahead", std_msgs.msg.Float32, self.on_collision_ahead_event)

        self.enabled = False
        self.state = DriverIdleState(self)
        self.sensor_state = SensorState()
        self.distance_to_next_obstacle = float("Inf")

        rospy.loginfo("Random driver successfully initialized!")

    def enable(self, msg):
        if not self.enabled:
            self.state = DriverForwardState(self)
        rospy.loginfo("Driver enabled")
        self.enabled = True

    def disable(self, msg):
        if self.enabled:
            self.state = DriverIdleState(self)
        rospy.loginfo("Driver disabled")
        self.enabled = False

    def on_collision_ahead_event(self, msg):
        self.distance_to_next_obstacle = msg.data

    def update(self, delta_time):
        self.state = self.state.update(delta_time)
