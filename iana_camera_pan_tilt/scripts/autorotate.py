#!/usr/bin/env python
import rospy
from iana_camera_pan_tilt.msg import PanTilt


class Autorotate(object):
    def __init__(self):
        rospy.init_node('autorotate', anonymous=True)
        self.set_pant_tilt_pub = rospy.Publisher('/iana/camera/set_pan_tilt', PanTilt, queue_size=1)

    def run(self):
        rate = rospy.Rate(10)
        angle_range = (40,150)
        pan = 0
        tilt = angle_range[0]
        tilt_direction = 1
        while not rospy.is_shutdown():
            self.set_pant_tilt_pub.publish(pan, tilt)
            tilt += tilt_direction
            if tilt == angle_range[0] or tilt == angle_range[1]:
                tilt_direction = -tilt_direction
            rate.sleep()


if __name__ == '__main__':
    try:
        autorotate = Autorotate()
        autorotate.run()
    except rospy.ROSInterruptException:
        pass
