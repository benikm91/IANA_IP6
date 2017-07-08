import rospy
import kobuki_msgs.msg

class SensorState(object):

    def  __init__(self):
        super(SensorState, self).__init__()

        self.bumper_pressed = [False, False, False]
        self.wheel_dropped = [False, False]
        self.cliff_detected = [False, False, False]

        rospy.Subscriber("/mobile_base/events/bumper", kobuki_msgs.msg.BumperEvent, self.on_bumper_event)
        rospy.Subscriber("/mobile_base/events/cliff", kobuki_msgs.msg.CliffEvent, self.on_cliff_event)
        rospy.Subscriber("/mobile_base/events/wheel_drop", kobuki_msgs.msg.WheelDropEvent, self.on_wheel_drop_event)

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