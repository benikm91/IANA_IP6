import cv2
import datetime
import numpy as np
from tesserocr import PyTessBaseAPI

import rospy
from iana_text_recognition.msg import TextWithROI
from sensor_msgs.msg import RegionOfInterest

import Image

from std_msgs.msg import UInt32, String, Header


class TextInImage:

    def __init__(self, text, position):
        self.text = text
        self._position = position

    @property
    def x(self):
        return self._position[0]

    @property
    def y(self):
        return self._position[1]

    @property
    def width(self):
        return self._position[2]

    @property
    def height(self):
        return self._position[3]


class TextDetection(object):

    def text_detection(self, image):

        morph_kernel_grad_shape = (2, 2)
        morph_kernel_close_shape = (9, 1)
        threshold_min_text_ratio = 0.5  # How much percent of the pixels are foreground
        threshold_min_height = 8  # Min height of bounding box
        threshold_min_width = 8  # Min width of bounding box

        morph_grad_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_grad_shape)
        morph_close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, morph_kernel_close_shape)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        grad = cv2.morphologyEx(gray, cv2.MORPH_GRADIENT, morph_grad_kernel)
        _, bw = cv2.threshold(grad, 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
        connected = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, morph_close_kernel)

        mask = np.zeros(bw.shape)
        _, contours, hierarchy = cv2.findContours(connected, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        hierarchy = hierarchy[0]  # why????

        result = list()

        idx = 0
        while idx >= 0:
            rect = cv2.boundingRect(contours[idx])
            mask_roi = mask[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]]
            cv2.drawContours(mask, contours, idx, (255, 255, 255), cv2.FILLED)
            r = (float(cv2.countNonZero(mask_roi)) / mask_roi.size) if mask_roi.size > 0 else 0
            if r > threshold_min_text_ratio and rect[2] > threshold_min_width and rect[3] > threshold_min_height:
                """
                assume at least 45% of the area is filled if it contains text
                constraints on region size:
                these two conditions alone are not very robust. better to use something
                like the number of significant peaks in a horizontal projection as a third condition
                """
                result.append(rect)
            idx = hierarchy[idx][0]

        return result


class TextRecognition(object):

    def __init__(self):
        self.language = 'deu'
        self.tesseract_configuration = {
            'user_words_suffix': 'user-words',
            #'tessedit_char_whitelist': 'MUSTAFA'
        }

    def text_recognition(self, image, global_position):
        with PyTessBaseAPI() as api:
            api.InitFull(lang=self.language, variables=self.tesseract_configuration)
            api.SetImage(Image.fromarray(image))
            result = api.GetUTF8Text()
            return TextInImage(result, global_position)


class TextRecognitionNode:

    def __init__(self, text_publisher):
        self.text_publisher = text_publisher
        self.text_detection = TextDetection()
        self.text_recognition = TextRecognition()

    def recognise_text(self, image):
        """
        :param image
        :type image numpy.array
        :return: .
        """
        def cutout_rectangles(img, rects):
            result = list()
            for rect in rects:
                result.append((img[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]], rect))
            return result

        def connect_texts(texts_in_image):
            """
            :param texts_in_image: 
            :type texts_in_image list[TextInImage]
            :return: 
            """
            connections = [-1 for _ in range(0, len(texts_in_image))]

            # find connections
            for i in range(0, len(texts_in_image)):
                for j in range(0, len(texts_in_image)):
                    if i != j:
                        image1 = texts_in_image[i]
                        image2 = texts_in_image[j]
                        if pow(image1.x - image2.x, 2) + pow(image1.y + image1.height - image2.y, 2) < 20:
                            connections[i] = j

            # apply connections
            for i in range(0, len(connections)):
                if connections[i] != -1:
                    if texts_in_image[i].text != "" and texts_in_image[connections[i]].text != "":
                        texts_in_image[i].text = texts_in_image[i].text + " " + texts_in_image[connections[i]].text
                        texts_in_image[connections[i]].text = ""

            return texts_in_image

        height, width, _ = image.shape

        positions = self.text_detection.text_detection(image)
        rospy.loginfo("Found {0} texts in current image".format(len(positions)))
        text_images = cutout_rectangles(image, positions)
        recognised_texts_in_image = [self.text_recognition.text_recognition(image, position) for image, position in text_images]
        connected_texts_in_image = connect_texts(recognised_texts_in_image)
        final_texts_in_image = connected_texts_in_image #filter(lambda text_in_image: text_in_image.text is None, connected_texts_in_image)
        rospy.loginfo("Found {0} connected texts in current image".format(len(final_texts_in_image)))

        if any(final_texts_in_image):
            self.text_publisher.publish(
                header=Header(stamp=rospy.Time.now()),
                origin_image_width=UInt32(width),
                origin_image_height=UInt32(height),
                texts_in_image=
                [
                    TextWithROI(
                        # TODO check if unicode necessary and possible in ROS ??
                        text=String(text_with_pos.text.encode('ascii', 'ignore')),
                        roi=RegionOfInterest(
                            x_offset=text_with_pos.x,
                            y_offset=text_with_pos.y,
                            width=text_with_pos.width,
                            height=text_with_pos.height
                        )
                    )
                    for text_with_pos in final_texts_in_image
                ]
            )
