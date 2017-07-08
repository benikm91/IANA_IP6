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

from scripts.common.frontier_detection import find_closest_frontier_point_in_occupancy_grid

class DriverExploreState(object):

    def __init__(self, driver):
        """
        :param driver:
        :type driver: DriverExplore
        """
        super(DriverExploreState, self).__init__()
        self.driver = driver

    def update(self, delta_time):
        raise NotImplementedError()

    def on_enable(self):
        raise NotImplementedError()

    def on_disable(self):
        raise NotImplementedError()

    def on_map_updated(self):
        raise NotImplementedError()


class ExploreFrontiersState(DriverExploreState):

    def __init__(self, driver):
        super(ExploreFrontiersState, self).__init__(driver)
        self.goal_pending = False
        self.move_base_action = actionlib.SimpleActionClient('/move_base', move_base_msgs.msg.MoveBaseAction)
        if not self.move_base_action.wait_for_server(rospy.Duration(30)):
            rospy.logerr('Failed to connect to /move_base action')

    def next_frontier_pose(self):
        if self.driver.occupancy_grid is None:
            return None
        closest_frontier_pose = find_closest_frontier_point_in_occupancy_grid(self.driver.occupancy_grid, 1)
        if closest_frontier_pose is None:
            return None
        return closest_frontier_pose

    def goal_reached(self, state, result):
        self.goal_pending = False

    def update(self, delta_time):
        if self.goal_pending is False:
            target_pose = self.next_frontier_pose()
            if target_pose is None:
                # can't find any valid frontier point!
                return ExploreRandomState(self.driver, 30)
            target_pose_stamped = geometry_msgs.msg.PoseStamped()
            target_pose_stamped.header.frame_id = 'map'
            target_pose_stamped.pose = target_pose
            goal = move_base_msgs.msg.MoveBaseGoal(target_pose=target_pose_stamped)
            self.move_base_action.send_goal(goal, self.goal_reached)
            self.goal_pending = True
        return self

    def on_enable(self):
        pass

    def on_disable(self):
        self.move_base_action.cancel_goal()
        self.goal_pending = False

    def on_map_updated(self):
        pass


class ExploreRandomState(DriverExploreState):
    def __init__(self, driver, duration):
        super(ExploreRandomState, self).__init__(driver)
        self.duration = duration
        self.driver_random_enable_publisher = rospy.Publisher('/iana/driver_random/enable', std_msgs.msg.Empty)
        self.driver_random_disable_publisher = rospy.Publisher('/iana/driver_random/disable', std_msgs.msg.Empty)
        self.driver_random_enable_publisher.publish()

    def update(self, delta_time):
        self.duration -= delta_time
        if self.duration <= 0:
            return ExploreRandomState(self.driver)
        return self

    def on_enable(self):
        self.driver_random_enable_publisher.publish()

    def on_disable(self):
        self.driver_random_disable_publisher.publish()

    def on_map_updated(self):
        pass