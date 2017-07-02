import math
import rospy
import std_msgs.msg
import geometry_msgs.msg
import kobuki_msgs.msg


class DriverRandomState(object):

    def __init__(self) -> None:
        super().__init__()

    def update(self):
        raise NotImplemented("not yet implemented!")


class DriveForwardState(DriverRandomState):
    def __init__(self, velocity) -> None:
        super().__init__()
        self.velocity = velocity

    def update(self):
        return super().update()


class DriverState(object):
    DRIVING_FORWARD, TURNING = range(2)


class DriverRandom(object):

    def __init__(self, node_handle):
        self.node_handle = node_handle

        self.enabled = False

        self.state = DriverState.DRIVING_FORWARD
        self.turning_duration = 0
        self.current_velocity = kobuki_msgs.msg.Twist()

        self.linear_velocity = rospy.get_param("linear_velocity", 0.5)
        self.angular_velocity = rospy.get_param("angular_velocity", 0.1)
        self.linear_acceleration = rospy.get_param("linear_acceleration", 0.1)
        self.angular_acceleration = rospy.get_param("angular_acceleration", 0.02)

        self.bumper_pressed = [False, False, False]
        self.wheel_dropped = [False, False]
        self.cliff_detected = [False, False, False]

        self.command_velocity_publisher = rospy.Publisher('/commands/velocity', geometry_msgs.msg.Twist, queue_size=10)

        rospy.Subscriber("/iana/driver_random/enable", Empty, self.enable)
        rospy.Subscriber("/iana/driver_random/disable", Empty, self.disable)

        rospy.Subscriber("/events/bumper", kobuki_msgs.msg.BumperEvent, self.on_bumper_event)
        rospy.Subscriber("/events/cliff", kobuki_msgs.msg.CliffEvent, self.on_cliff_event)
        rospy.Subscriber("/events/wheel_drop", kobuki_msgs.msg.WheelDropEvent, self.on_wheel_drop_event)

    def enable(self, msg):
        self.enabled = True

    def disable(self, msg):
        self.enabled = False

    def on_bumper_event(self, msg):
        is_pressed = False
        if msg.state == kobuki_msgs.msg.BumperEvent.PRESSED:
            is_pressed = True

        if msg.bumper == kobuki_msgs.msg.BumperEvent.LEFT:
            self.bumper_pressed[0] = is_pressed
        elif msg.bumper == kobuki_msgs.msg.BumperEvent.CENTER:
            self.bumper_pressed[1] = is_pressed
        else:
            self.bumper_pressed[2] = is_pressed

        rospy.loginfo("received bumper event: {}".format(msg))

    def on_cliff_event(self, msg):
        is_cliff = False
        if msg.sensor == kobuki_msgs.msg.CliffEvent.CLIFF:
            is_cliff = True

        if msg.sensor == kobuki_msgs.msg.CliffEvent.LEFT:
            self.cliff_detected[0] = is_cliff
        elif msg.sensor == kobuki_msgs.msg.CliffEvent.CENTER:
            self.cliff_detected[1] = is_cliff
        else:
            self.cliff_detected[2] = is_cliff

        rospy.loginfo("received bumper event: {}".format(msg))

    def on_wheel_drop_event(self, msg):
        is_dropped = False
        if msg.state == kobuki_msgs.msg.WheelDropEvent.DROPPED:
            is_dropped = True

        if msg.wheel == kobuki_msgs.msg.WheelDropEvent.LEFT:
            self.wheel_dropped[0] = is_dropped
        else:
            self.wheel_dropped[1] = is_dropped

        rospy.loginfo("received wheel drop event: {}".format(msg))

    def update(self, delta_time):
        if self.enabled:
            if any(self.cliff_detected) or any(self.wheel_dropped):
                # emergency exit!!!!! D:
                rospy.logerr("Heeeelp!!")
                return

            if any(self.bumper_pressed) and self.state != DriverState.TURNING:
                # run into something: make a ~180° turn and go on
                self.turning_duration = math.pi / self.angular_velocity
                self.state = DriverState.TURNING

            if self.state == DriverState.DRIVING_FORWARD:
                if now < self.turning_duration:
                    self.command_velocity_publisher.publish()
            else:
                pass



