#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

bridge = CvBridge()


def publish_images():
    pub = rospy.Publisher('image', Image, queue_size=1)
    rospy.init_node('url_wrapper', anonymous=True, log_level=rospy.INFO)
    rate = rospy.Rate(1)

    rospy.loginfo("setup started")
    url = rospy.get_param("url", "http://admin:@192.168.0.20:80/video1.mjpg")

    video_capture = cv2.VideoCapture(url)
    pos_frame = video_capture.get(1)

    rospy.loginfo("setup done")

    while not rospy.is_shutdown():
        ret, frame = video_capture.read()
        if ret:
            rospy.loginfo("Next image")
            image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            image_message.header.stamp = rospy.Time.now()
            pub.publish(image_message)
        else:
            rospy.logwarn("No next image since last time")
            # The next frame is not ready, so we try to read it again
            video_capture.set(cv2.CAP_PROP_POS_FRAMES, pos_frame - 1)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_images()
    except rospy.ROSInterruptException:
        pass
