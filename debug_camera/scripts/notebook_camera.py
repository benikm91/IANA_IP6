#!/usr/bin/env python

import rospy

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


capture_device = 0
width = 800
height = 600

video_capture = cv2.VideoCapture(capture_device)
video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
video_capture.set(3, width)
video_capture.set(4, height)
interrupted = False
pos_frame = video_capture.get(1)

print pos_frame

bridge = CvBridge()


def publish_images():
    pub = rospy.Publisher('image', Image, queue_size=1)
    rospy.init_node('notebook_camera', anonymous=True)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():

        # Linux buffers 5 frames from a video capture device. So to get the newest, we retrieve the 5 "newest" frames.
        ret, frame = False, None
        for i in range(0, 5):
            ret, frame = video_capture.read()

        if ret:
            image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            image_message.header.stamp = rospy.Time.now()
            pub.publish(image_message)
        else:
            # The next frame is not ready, so we try to read it again
            video_capture.set(cv2.CAP_PROP_POS_FRAMES, pos_frame - 1)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_images()
    except rospy.ROSInterruptException:
        pass
