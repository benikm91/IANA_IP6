import math
import random

import numpy as np
import rospy
import geometry_msgs.msg

ZERO_VELOCITY = geometry_msgs.msg.Twist()

class DriverRandomState(object):

    def __init__(self, driver):
        """
        :type driver: DriverRandom
        """
        super(DriverRandomState, self).__init__()
        self.driver = driver

    def _is_collision_ahead(self):
        return any(self.driver.sensor_state.bumper_pressed) or \
               any(self.driver.sensor_state.cliff_detected) or \
               any(self.driver.sensor_state.wheel_dropped) or \
               self.driver.distance_to_next_obstacle <= self.driver.collision_ahead_threshold

    def update(self, delta_time):
        raise NotImplemented("not yet implemented!")


class DriverForwardState(DriverRandomState):

    def __init__(self, driver, current_velocity=0):
        super(DriverForwardState, self).__init__(driver)
        self.current_velocity = current_velocity
        self.target_velocity = self.driver.max_linear_velocity

    def update(self, delta_time):
        # set target velocity depending on safety of robot
        if self.driver.distance_to_next_obstacle <= self.driver.collision_warn_threshold:
            self.target_velocity = self.driver.min_linear_velocity
        elif self.driver.distance_to_next_obstacle <= self.driver.collision_info_threshold:
            self.target_velocity = self.driver.medium_linear_velocity
        else:
            self.target_velocity = self.driver.max_linear_velocity

        # handle collision events
        if self._is_collision_ahead():
            self.driver.command_velocity_publisher.publish(ZERO_VELOCITY)
            rospy.loginfo("Collision ahead! go to turning state")
            return DriverTurningState(self.driver, random.uniform(math.pi/6, math.pi), np.random.choice([-1, 1]))

        # if target velocity not reached yet: accelerate!
        if self.current_velocity < self.target_velocity:
            self.current_velocity = min(
                self.current_velocity + self.driver.linear_acceleration * delta_time,
                self.target_velocity
            )
        elif self.current_velocity > self.target_velocity:
            self.current_velocity = max(
                self.current_velocity - self.driver.linear_acceleration * delta_time,
                self.target_velocity
            )

        # publish new velocity
        new_velocity = geometry_msgs.msg.Twist()
        new_velocity.linear.x = self.current_velocity
        rospy.loginfo("ForwardState, Publishing new velocity: {}".format(new_velocity))
        self.driver.command_velocity_publisher.publish(new_velocity)
        return self


class DriverTurningState(DriverRandomState):

    def __init__(self, driver, target_angle, direction, current_velocity=0):
        super(DriverTurningState, self).__init__(driver)
        self.target_angle = target_angle
        self.turned_angle = 0.0
        self.direction = direction
        self.current_velocity = current_velocity
        self.target_velocity = self.driver.max_linear_velocity

    def update(self, delta_time):
        # update
        self.turned_angle += delta_time * self.current_velocity

        # handle target angle reached
        if self.target_angle <= self.turned_angle:
            if self._is_collision_ahead():
                rospy.loginfo("Goal reached, still obstacles!")
                self.target_angle = random.uniform(math.pi/6, math.pi)
                self.turned_angle = 0.0
            else:
                self.driver.command_velocity_publisher.publish(ZERO_VELOCITY)
                rospy.loginfo("Turning is ova! Back to Forward state")
                return DriverForwardState(self.driver, 0)

        # set target velocity depending on distance to goal
        if (self.target_angle - self.turned_angle) < math.pi/6:
            self.target_velocity = self.driver.min_angular_velocity
        else:
            self.target_velocity = self.driver.max_angular_velocity

        # if target velocity not reached yet: accelerate!
        if self.current_velocity < self.target_velocity:
            self.current_velocity = min(
                self.current_velocity + self.driver.angular_acceleration * delta_time,
                self.target_velocity
            )
        elif self.current_velocity > self.target_velocity:
            self.current_velocity = max(
                self.current_velocity - self.driver.angular_acceleration * delta_time,
                self.target_velocity
            )

        # publish new velocity
        new_velocity = geometry_msgs.msg.Twist()
        new_velocity.angular.z = self.direction * self.current_velocity
        rospy.loginfo("TurningState, Publishing new velocity: {}".format(new_velocity))
        self.driver.command_velocity_publisher.publish(new_velocity)
        return self


class DriverIdleState(DriverRandomState):

    def __init__(self, driver):
        super(DriverIdleState, self).__init__(driver)
        self.driver.command_velocity_publisher.publish(ZERO_VELOCITY)

    def update(self, delta_time):
        rospy.loginfo("IdleState")
        return self
