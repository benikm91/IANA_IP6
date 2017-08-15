#!/usr/bin/env python
# coding=utf-8
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
        x, y = text_in_image.roi.x_offset, text_in_image.roi.y_offset
        width, height = text_in_image.roi.width, text_in_image.roi.height
        pt1 = (x, y)
        pt2 = (x + width, y + height)
        cv2.rectangle(image, pt1, pt2, (0, 255, 0), 5)
        # Write some Text
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = text_in_image.text.data
        cv2.putText(image, text, (x, y), font, 1, (255, 255, 255), 2)

    publisher.publish(bridge.cv2_to_imgmsg(image))


if __name__ == '__main__':
    try:
        rospy.init_node('combine_image_and_recognition', anonymous=True)
        image = message_filters.Subscriber("/image", Image)
        debug_image_publisher = rospy.Publisher("/debug_image", Image, queue_size=1)
        texts_in_image = message_filters.Subscriber("/texts_in_image", TextsInImage)

        message_filters.TimeSynchronizer([image, texts_in_image], 100).registerCallback(combine, debug_image_publisher)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass