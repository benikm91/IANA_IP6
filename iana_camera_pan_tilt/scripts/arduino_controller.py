#!/usr/bin/env python
import serial
import struct
import rospy
from iana_camera_pan_tilt.msg import PanTilt


class ArduinoController(object):
    def __init__(self):
        rospy.init_node('arduino_controller', anonymous=True)
        rospy.Subscriber("set_pan_tilt", PanTilt, self.handle_set_pan_tilt, queue_size=1)
        self.pant_tilt_pub = rospy.Publisher('pan_tilt', PanTilt, queue_size=10)
        self.serial = serial.Serial('/dev/ttyACM0', 9600)
        self.state = [135, 90]

    def run(self):
        self.__set_pan_tilt(self.state[0], self.state[1])
        rospy.spin()

    def handle_set_pan_tilt(self, pan_tilt):
        self.__set_pan_tilt(pan_tilt.pan, pan_tilt.tilt)

    def __set_pan_tilt(self, pan, tilt):
        pan = min(180, max(0, pan))
        tilt = min(180, max(0, tilt))
        state = [pan, tilt]
        rospy.logerr("received    xpan: {}, tilt: {}".format(pan, tilt))
        self.serial.write(bytearray(struct.pack('I' * len(state), *state)))
        self.pant_tilt_pub.publish(pan, tilt)
        rospy.logerr("transmitted pan: {}, tilt: {}".format(pan, tilt))


if __name__ == '__main__':
    try:
        arduino_controller = ArduinoController()
        arduino_controller.run()
    except rospy.ROSInterruptException:
        pass
