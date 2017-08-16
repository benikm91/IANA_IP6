#!/usr/bin/env python
import serial
import struct
import rospy
import time
from iana_camera_pan_tilt.msg import PanTilt


class ArduinoController(object):
    def __init__(self):
        self.state = [90, 90]
        rospy.init_node('arduino_controller', anonymous=True)

        self.serial_port = rospy.get_param("serial_port", '/dev/ttyACM0')
        self.serial_baudrate = rospy.get_param("serial_baudrate", 9600)
        self.write_timeout = rospy.get_param("write_timeout", 3)
        self.read_timeout = rospy.get_param("read_timeout", 1)

        rospy.Subscriber("/iana/camera/set_pan_tilt", PanTilt, self.handle_set_pan_tilt, queue_size=5)

        self.pant_tilt_pub = rospy.Publisher('/iana/camera/pan_tilt', PanTilt, queue_size=10)

        self.serial = serial.Serial(self.serial_port, baudrate=self.serial_baudrate)

    def run(self):
        self.__set_pan_tilt(self.state[0], self.state[1])
        rospy.spin()

    def handle_set_pan_tilt(self, pan_tilt):
        self.__set_pan_tilt(pan_tilt.pan, pan_tilt.tilt)

    def __set_pan_tilt(self, pan, tilt):
        pan = min(180, max(1, pan))
        tilt = min(180, max(1, tilt))
        self.state = [pan, tilt]
        rospy.loginfo("set pan: {}, tilt: {}".format(pan, tilt))
        success = False
        start_time = time.time()
        while not success and (time.time() - start_time) < self.write_timeout:
            self.serial.timeout = self.write_timeout - (time.time() - start_time)
            self.serial.write(bytearray(struct.pack('I' * len(self.state), *self.state)))
            if self.serial.in_waiting <= 1:
                self.serial.timeout = self.read_timeout
                try:
                    result = self.serial.read(1)
                    success = len(result) == 1 and result[0] == 0
                except serial.SerialException:
                    rospy.loginfo("ignore SerialException on read")
            else:
                self.serial.reset_input_buffer()
        if success:
            self.pant_tilt_pub.publish(pan, tilt)


if __name__ == '__main__':
    try:
        arduino_controller = ArduinoController()
        arduino_controller.run()
    except rospy.ROSInterruptException:
        pass
