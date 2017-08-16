#!/usr/bin/env python
import serial
import struct
import rospy
import time
from iana_camera_pan_tilt.msg import PanTilt


class ArduinoController(object):
    def __init__(self):
        rospy.init_node('arduino_controller', anonymous=True)

        self.initial_pan = rospy.get_param("initial_pan", 90)
        self.initial_tilt = rospy.get_param("initial_tilt", 90)
        self.serial_port = rospy.get_param("serial_port", '/dev/ttyACM0')
        self.serial_baudrate = rospy.get_param("serial_baudrate", 9600)
        self.init_timeout = rospy.get_param("init_timeout", 5)
        self.write_timeout = rospy.get_param("write_timeout", 3)
        self.read_timeout = rospy.get_param("read_timeout", 1)
        self.state = [self.initial_pan, self.initial_tilt]

        success = False
        start_time = time.time()
        while not success and (time.time() - start_time) < self.init_timeout:
            try:
                self.serial = serial.Serial(self.serial_port, baudrate=self.serial_baudrate)
                success = True
            except serial.serialutil.SerialException:
                pass
        if not success:
            raise Exception("Failed to connect to arduino on port '{}'".format(self.serial_port))

        self.pant_tilt_sub = rospy.Subscriber("/iana/camera/set_pan_tilt", PanTilt, self.handle_set_pan_tilt, queue_size=5)
        self.pant_tilt_pub = rospy.Publisher('/iana/camera/pan_tilt', PanTilt, queue_size=10)

    def run(self):
        self.__set_pan_tilt(self.state[0], self.state[1])
        rospy.spin()

    def handle_set_pan_tilt(self, pan_tilt):
        self.__set_pan_tilt(pan_tilt.pan, pan_tilt.tilt)

    def __set_pan_tilt(self, pan, tilt):
        pan = min(180, max(1, pan))
        tilt = min(180, max(1, tilt))
        self.state = [pan, tilt]
        rospy.loginfo("Set pan: {}, tilt: {}".format(pan, tilt))
        success = False
        start_time = time.time()
        self.serial.reset_input_buffer()
        while not success and (time.time() - start_time) < self.write_timeout:
            self.serial.timeout = self.write_timeout - (time.time() - start_time)
            try:
                self.serial.write(bytearray(struct.pack('I' * len(self.state), *self.state)))
            except serial.SerialTimeoutException:
                rospy.logwarn("SerialTimeoutException while writing to serial port")
            if self.serial.in_waiting <= 1:
                self.serial.timeout = self.read_timeout
                try:
                    result = self.serial.read(1)
                    if len(result) > 0:
                        result_code = int(result.encode('hex'))
                        if result_code == 0:
                            success = True
                            rospy.logwarn("Received return code: {}".format(result_code))
                        else:
                            rospy.logwarn("Received error code: {}".format(result_code))
                    else:
                        rospy.logwarn("Received no result code")
                except serial.SerialException:
                    rospy.loginfo("Ignore SerialException while reading from serial port")
            else:
                self.serial.reset_input_buffer()
        if success:
            self.pant_tilt_pub.publish(pan, tilt)
            rospy.loginfo("Successfully sent pan & tilt values to arduino")
        else:
            rospy.logerr("Failed to sent pan & tilt values to arduino")


if __name__ == '__main__':
    try:
        arduino_controller = ArduinoController()
        arduino_controller.run()
    except rospy.ROSInterruptException:
        pass
