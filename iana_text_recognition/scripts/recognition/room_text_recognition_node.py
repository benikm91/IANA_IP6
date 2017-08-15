import rospy
from iana_text_recognition.msg import TextWithROI
from recognition.room_text_detection import RoomNameDetection, RoomNumberDetection
from recognition.room_text_recognition import RoomNumberRecognition, RoomNameRecognition
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Header, UInt32, String


class RoomTextRecognitionNode:

    def __init__(self, text_publisher):
        self.text_publisher = text_publisher
        self.name_detection = RoomNameDetection(
            rospy.get_param('~name_detection/kernel_grad_shape', (2, 2)),
            rospy.get_param('~name_detection/kernel_close_shape', (20, 10)),
            rospy.get_param('~name_detection/threshold_min_text_ratio',  0.5),
            rospy.get_param('~name_detection/threshold_min_height',  15),
            rospy.get_param('~name_detection/threshold_min_width',  50)
        )
        self.number_detection = RoomNumberDetection(
            rospy.get_param('~number_detection/kernel_remove_background_shape', (10, 10)),
            rospy.get_param('~number_detection/kernel_grad_shape', (2, 2)),
            rospy.get_param('~number_detection/kernel_close_shape', (100, 50)),
            rospy.get_param('~number_detection/threshold_min_text_ratio', 0.5),
            rospy.get_param('~number_detection/threshold_min_height', 80),
            rospy.get_param('~number_detection/threshold_min_width', 40 )
        )
        self.name_recognition = RoomNameRecognition(
            rospy.get_param('~language', 'deu')
        )
        self.number_recognition = RoomNumberRecognition(
            rospy.get_param('~language', 'deu'),
            rospy.get_param('~number_detection/kernel_remove_background_shape', (10, 10)),
            rospy.get_param('~number_detection/kernel_thin_text_shape', (10, 10)),
        )

    def recognise_text(self, img, time_stamp):
        """
        :param image
        :type image numpy.array
        :return: .
        """

        height, width, _ = img.shape

        def detect_and_recognise(img, detector, recognisor):
            text_positions = detector.text_detection(img)
            rospy.logdebug("Found {0} name text positions in current image".format(len(text_positions)))
            result = recognisor.texts_recognition(img, text_positions)
            rospy.logdebug("Found {0} connected texts in current image".format(len(result)))
            return result

        texts = detect_and_recognise(img, self.name_detection, self.name_recognition) + \
                detect_and_recognise(img, self.number_detection, self.number_recognition)

        for t in texts:
            print t.text

        self.text_publisher.publish(
            header=Header(stamp=time_stamp),
            origin_image_width=UInt32(width),
            origin_image_height=UInt32(height),
            texts_in_image=
            [
                TextWithROI(
                    text=String(text_with_pos.text.strip().encode('utf-8', 'ignore')),
                    roi=RegionOfInterest(
                        x_offset=text_with_pos.x,
                        y_offset=text_with_pos.y,
                        width=text_with_pos.width,
                        height=text_with_pos.height
                    )
                )
                for text_with_pos in texts
            ]
        )
