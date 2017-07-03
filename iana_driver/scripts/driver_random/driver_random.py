import math
import random

import numpy as np
import rospy
import std_msgs.msg
import geometry_msgs.msg
import kobuki_msgs.msg

ZERO_VELOCITY = geometry_msgs.msg.Twist()


class DriverRandom(object):

    def __init__(self):
        rospy.loginfo("Starting random driver...")

        self.min_linear_velocity = rospy.get_param("min_linear_velocity", 0.1)
        self.max_linear_velocity = rospy.get_param("max_linear_velocity", 0.5)
        self.min_angular_velocity = rospy.get_param("min_angular_velocity", 0.05)
        self.max_angular_velocity = rospy.get_param("max_angular_velocity", 0.2)
        self.linear_acceleration = rospy.get_param("linear_acceleration", 0.1)
        self.angular_acceleration = rospy.get_param("angular_acceleration", 0.02)
        self.collision_warn_threshold = rospy.get_param("collision_warn_threshold", 1.3)
        self.collision_ahead_threshold = rospy.get_param("collision_ahead_threshold", 0.8)

        self.command_velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', geometry_msgs.msg.Twist, queue_size=10)

        rospy.Subscriber("/iana/driver_random/enable", std_msgs.msg.Empty, self.enable)
        rospy.Subscriber("/iana/driver_random/disable", std_msgs.msg.Empty, self.disable)
        rospy.Subscriber("/mobile_base/events/bumper", kobuki_msgs.msg.BumperEvent, self.on_bumper_event)
        rospy.Subscriber("/mobile_base/events/cliff", kobuki_msgs.msg.CliffEvent, self.on_cliff_event)
        rospy.Subscriber("/mobile_base/events/wheel_drop", kobuki_msgs.msg.WheelDropEvent, self.on_wheel_drop_event)
        rospy.Subscriber("/collision_ahead", std_msgs.msg.Float32, self.on_collision_ahead_event)

        self.enabled = False
        self.sensor_state = SensorState()
        self.state = DriverIdleState(self)

        rospy.loginfo("Random driver successfully!")

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

    def on_bumper_event(self, msg):
        is_pressed = False
        if msg.state == kobuki_msgs.msg.BumperEvent.PRESSED:
            is_pressed = True

        if msg.bumper == kobuki_msgs.msg.BumperEvent.LEFT:
            self.sensor_state.bumper_pressed[0] = is_pressed
        elif msg.bumper == kobuki_msgs.msg.BumperEvent.CENTER:
            self.sensor_state.bumper_pressed[1] = is_pressed
        else:
            self.sensor_state.bumper_pressed[2] = is_pressed

        rospy.loginfo("received bumper event: {}".format(msg))

    def on_cliff_event(self, msg):
        is_cliff = False
        if msg.sensor == kobuki_msgs.msg.CliffEvent.CLIFF:
            is_cliff = True

        if msg.sensor == kobuki_msgs.msg.CliffEvent.LEFT:
            self.sensor_state.cliff_detected[0] = is_cliff
        elif msg.sensor == kobuki_msgs.msg.CliffEvent.CENTER:
            self.sensor_state.cliff_detected[1] = is_cliff
        else:
            self.sensor_state.cliff_detected[2] = is_cliff

        rospy.loginfo("received bumper event: {}".format(msg))

    def on_wheel_drop_event(self, msg):
        is_dropped = False
        if msg.state == kobuki_msgs.msg.WheelDropEvent.DROPPED:
            is_dropped = True

        if msg.wheel == kobuki_msgs.msg.WheelDropEvent.LEFT:
            self.sensor_state.wheel_dropped[0] = is_dropped
        else:
            self.sensor_state.wheel_dropped[1] = is_dropped

        rospy.loginfo("received wheel drop event: {}".format(msg))

    def on_collision_ahead_event(self, msg):
        self.sensor_state.collision_warn = msg.data < self.collision_warn_threshold
        self.sensor_state.collision_ahead = msg.data < self.collision_ahead_threshold

        if self.sensor_state.collision_ahead:
            rospy.loginfo("Collision ahead! ({})".format(msg.data))

    def update(self, delta_time):
        self.state = self.state.update(delta_time)


class SensorState(object):
    def __init__(self):
        super(SensorState, self).__init__()
        self.bumper_pressed = [False, False, False]
        self.wheel_dropped = [False, False]
        self.cliff_detected = [False, False, False]
        self.collision_warn = False
        self.collision_ahead = False


class DriverRandomState(object):

    def __init__(self, driver):
        """
        :type driver: DriverRandom
        """
        super(DriverRandomState, self).__init__()
        self.driver = driver

    def update(self, delta_time):
        raise NotImplemented("not yet implemented!")


class DriverForwardState(DriverRandomState):

    def __init__(self, driver, current_velocity=0):
        super(DriverForwardState, self).__init__(driver)
        self.current_velocity = current_velocity
        self.target_velocity = self.driver.max_linear_velocity

    def update(self, delta_time):
        # set target velocity depending on safety of robot
        if self.driver.sensor_state.collision_warn:
            self.target_velocity = self.driver.min_linear_velocity
        else:
            self.target_velocity = self.driver.max_linear_velocity

        # handle collision events
        if any(self.driver.sensor_state.bumper_pressed) or \
                any(self.driver.sensor_state.cliff_detected) or \
                any(self.driver.sensor_state.wheel_dropped) or \
                self.driver.sensor_state.collision_ahead:
            self.driver.command_velocity_publisher.publish(ZERO_VELOCITY)
            rospy.logerr("Collision ahead! go to turning state")
            return DriverTurningState(self.driver, random.uniform(30, 180), np.random.choice([-1, 1]))

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
        rospy.logerr("ForwardState, Publishing new velocity: {}".format(new_velocity))
        self.driver.command_velocity_publisher.publish(new_velocity)
        return self


class DriverTurningState(DriverRandomState):

    def __init__(self, driver, target_angle, direction, current_velocity=0):
        super(DriverTurningState, self).__init__(driver)
        self.target_angle = target_angle
        self.turned_angle = 0
        self.direction = direction
        self.current_velocity = current_velocity
        self.target_velocity = self.driver.max_linear_velocity

    def update(self, delta_time):
        # update
        self.turned_angle += delta_time * self.current_velocity

        # handle target angle reached
        if self.target_angle <= self.turned_angle:
            if any(self.driver.sensor_state.bumper_pressed) or \
                    any(self.driver.sensor_state.cliff_detected) or \
                    any(self.driver.sensor_state.wheel_dropped) or \
                    self.driver.sensor_state.collision_ahead:
                self.target_angle = random.uniform(30, 180)
                self.turned_angle = 0
            else:
                self.driver.command_velocity_publisher.publish(ZERO_VELOCITY)
                rospy.logerr("Turning is ova! Back to Forward state")
                return DriverForwardState(0)

        # set target velocity depending on distance to goal
        if (self.target_angle - self.turned_angle) < 10:
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
        rospy.logerr("TurningState, Publishing new velocity: {}".format(new_velocity))
        self.driver.command_velocity_publisher.publish(new_velocity)
        return self


class DriverIdleState(DriverRandomState):

    def __init__(self, driver):
        super(DriverIdleState, self).__init__(driver)
        self.driver.command_velocity_publisher.publish(ZERO_VELOCITY)

    def update(self, delta_time):
        rospy.logerr("IdleState!")
        return self


