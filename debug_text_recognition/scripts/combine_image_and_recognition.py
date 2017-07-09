#!/usr/bin/env python
import cv2
import rospy
import message_filters
from iana_text_recognition.msg import TextsInImage
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
bridge = CvBridge()


def combine(image_msg, texts_in_image_msg, publisher):
    image = bridge.imgmsg_to_cv2(image_msg)
    for text_in_image in texts_in_image_msg.texts_in_image:
        pt1 = (text_in_image.roi.x_offset, text_in_image.roi.y_offset)
        pt2 = (pt1[0] + text_in_image.roi.width, pt1[1] + text_in_image.roi.height)
        cv2.rectangle(image, pt1, pt2, (0, 255, 0), 5)
    publisher.publish(bridge.cv2_to_imgmsg(image))


if __name__ == '__main__':
    try:
        rospy.init_node('combine_image_and_recognition', anonymous=True)
        image = message_filters.Subscriber("/image", Image)
        debug_image_publisher = rospy.Publisher("/debug_image", Image, queue_size=1)
        texts_in_image = message_filters.Subscriber("/texts_in_image", TextsInImage)

        message_filters.ApproximateTimeSynchronizer([image, texts_in_image], 1, 500).registerCallback(combine, debug_image_publisher)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass