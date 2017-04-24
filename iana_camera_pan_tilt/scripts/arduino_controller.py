#!/usr/bin/env python3
# license removed for brevity
import serial
import struct
import rospy
from iana_camera_pan_tilt.msg import PanTilt


class ArduinoController(object):
    def __init__(self):
        rospy.init_node('arduino_controller', anonymous=True)
        rospy.Subscriber("set_pan_tilt", PanTilt, self.handle_set_pan_tilt)
        self.pant_tilt_pub = rospy.Publisher('pan_tilt', PanTilt, queue_size=10)
        self.serial = serial.Serial('/dev/ttyACM0', 9600)
        self.state = [90, 90]

    def run(self):
        self.pant_tilt_pub.publish(self.state[0], self.state[1])
        rospy.spin()
        #rate = rospy.Rate(10)  # 10hz
        #while not rospy.is_shutdown():
        #    rate.sleep()

    def handle_set_pan_tilt(self, pan_tilt: PanTilt) -> None:
        self.__set_pan_tilt(pan_tilt.pan, pan_tilt.tilt)

    def __set_pan_tilt(self, pan: int, tilt: int) -> None:
        pan = min(180, max(0, pan))
        tilt = min(180, max(0, tilt))
        state = [pan, tilt]
        self.serial.write(bytearray(struct.pack('%sI' % len(state), *state)))
        self.pant_tilt_pub.publish(pan, tilt)


if __name__ == '__main__':
    try:
        arduino_controller = ArduinoController()
        arduino_controller.run()
    except rospy.ROSInterruptException:
        pass
