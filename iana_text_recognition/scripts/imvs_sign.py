#!/usr/bin/env python
import cv2
import rospy
from recognition.TextRecognitionNode import TextRecognitionNode

from sensor_msgs.msg import Image

from iana_text_recognition.msg import TextsInImage

from cv_bridge import CvBridge
bridge = CvBridge()

if __name__ == '__main__':
    try:
        rospy.init_node('imvs_sign', anonymous=True)
        publisher = rospy.Publisher('/texts_in_image', TextsInImage, queue_size=1)

        def detect_text(msg, text_recognition):

            def get_image(message, encoding):
                message.encoding = "bgr8"
                frame = bridge.imgmsg_to_cv2(message, desired_encoding=encoding)
                return frame

            text_recognition_image = get_image(msg, "bgr8")
            text_recognition.recognise_text(text_recognition_image, msg.header.stamp)

        rospy.Subscriber("/image", Image, detect_text, TextRecognitionNode(publisher), queue_size = 1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
