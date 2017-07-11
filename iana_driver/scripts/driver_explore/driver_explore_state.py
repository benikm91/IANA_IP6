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

from common.frontier_detection import find_closest_frontier_point_in_occupancy_grid, \
    find_random_frontier_point_in_occupancy_grid


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
        return self

    def on_disable(self):
        return self

    def on_map_updated(self):
        return self

    def on_odometry_updated(self):
        return self


class ExploreFrontiersState(DriverExploreState):

    def __init__(self, driver):
        super(ExploreFrontiersState, self).__init__(driver)
        print('Init Explore Frontier state...')
        self.goal_pending = False
        self.move_base_action = actionlib.SimpleActionClient('/move_base', move_base_msgs.msg.MoveBaseAction)
        if not self.move_base_action.wait_for_server(rospy.Duration(30)):
            rospy.logerr('Failed to connect to /move_base action')
        print('Init Explore Frontier state done!')

    def next_frontier_pose(self):
        print('Find next frontier point...')
        if self.driver.occupancy_grid is None:
            print('Occupancy grid empty!')
            return None
        return find_random_frontier_point_in_occupancy_grid(self.driver.occupancy_grid, self.driver.odometry, 1)
        #return find_closest_frontier_point_in_occupancy_grid(self.driver.occupancy_grid, self.driver.odometry, 1)

    def goal_reached(self, state, result):
        print('Explore Frontier goal reached!')
        self.goal_pending = False

    def update(self, delta_time):
        if self.goal_pending is False:
            target_pose = self.next_frontier_pose()
            if target_pose is None:
                print('Can\'t find target pose: go to random state')
                # can't find any valid frontier point!
                return ExploreRandomState(self.driver, 30)
            target_pose_stamped = geometry_msgs.msg.PoseStamped()
            target_pose_stamped.header.frame_id = 'map'
            target_pose_stamped.pose = target_pose
            print('New goal: {}'.format(target_pose_stamped))
            goal = move_base_msgs.msg.MoveBaseGoal(target_pose=target_pose_stamped)
            self.move_base_action.send_goal(goal, self.goal_reached)
            self.goal_pending = True
        return self

    def on_disable(self):
        print('Driver frontier disabled')
        self.move_base_action.cancel_goal()
        self.goal_pending = False
        return IdleState(self.driver)


class ExploreRandomState(DriverExploreState):
    def __init__(self, driver, duration):
        super(ExploreRandomState, self).__init__(driver)
        print('Init Explore Random state...')
        self.duration = duration
        self.driver_random_enable_publisher = rospy.Publisher('/iana/driver_random/enable', std_msgs.msg.Empty, queue_size=10)
        self.driver_random_disable_publisher = rospy.Publisher('/iana/driver_random/disable', std_msgs.msg.Empty, queue_size=10)
        self.driver_random_enable_publisher.publish()
        print('Init Explore Random state done!')

    def update(self, delta_time):
        self.duration -= delta_time
        if self.duration <= 0:
            print('Explore time expired: go to Explore Frontier state.')
            self.driver_random_disable_publisher.publish()
            return ExploreFrontiersState(self.driver)
        return self

    def on_disable(self):
        self.driver_random_disable_publisher.publish()
        return IdleState(self.driver)


class IdleState(DriverExploreState):
    def __init__(self, driver):
        super(IdleState, self).__init__(driver)

    def update(self, delta_time):
        return self

    def on_enable(self):
        return ExploreFrontiersState(self.driver)
