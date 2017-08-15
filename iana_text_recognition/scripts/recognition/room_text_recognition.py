import Image
import re
from tesserocr import PyTessBaseAPI

import cv2
import numpy as np


# http://www.pyimagesearch.com/2017/02/20/text-skew-correction-opencv-python/
def deskew(img):
    def get_deskew_angle():
        coords = np.column_stack(np.where(img == 0))
        angle = cv2.minAreaRect(coords)[-1]

        if angle < -45:
            angle = -(90 + angle)
        else:
            angle = -angle
        return angle

    (h, w) = img.shape[:2]
    M = cv2.getRotationMatrix2D((w // 2, h // 2), get_deskew_angle(), 1.0)
    img = cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
    return img


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

    @property
    def position(self):
        return self._position


class TextRecognition(object):

    @staticmethod
    def _cutout_rectangles(img, rects):
        result = list()
        height, width = img.shape[:2]
        for rect in rects:
            delta = 5
            x = max(rect[1] - delta, 0)
            xx = min(rect[1] + rect[3] + delta, width - 1)
            y = max(rect[0] - delta, 0)
            yy = min(rect[0] + rect[2] + delta, height - 1)
            result.append((img[x:xx, y:yy], rect))
        return result

    def __init__(self, language):
        self.language = language
        self.tesseract_configuration = {
            'user_words_suffix': 'user-words',
        }

    def _pre_processing_on_text_img(self, img):
        return img

    def _post_processing_on_texts(self, texts):
        return texts

    def _text_recognition(self, img):
        with PyTessBaseAPI() as api:
            api.InitFull(lang=self.language, variables=self.tesseract_configuration)
            api.SetImage(Image.fromarray(img))
            return api.GetUTF8Text()

    def texts_recognition(self, img, positions):
        text_images = self._cutout_rectangles(img, positions)
        result = list()
        for img, position in text_images:
            img = self._pre_processing_on_text_img(img)
            result.append(TextInImage(self._text_recognition(img), position))
        return self._post_processing_on_texts(result)


class RoomNameRecognition(TextRecognition):

    def __init__(self, language):
        super(RoomNameRecognition, self).__init__(language)
        self.language = language

    def _connect_nearby_name_texts(self, texts_in_image):
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
                    if pow(image1.x - image2.x, 2) + pow(image1.y + image1.height - image2.y, 2) < 200:
                        connections[i] = j

        # apply connections
        for i in range(0, len(connections)):
            if connections[i] != -1:
                if texts_in_image[i].text != "" and texts_in_image[connections[i]].text != "":
                    texts_in_image[i].text = texts_in_image[i].text + " " + texts_in_image[connections[i]].text
                    texts_in_image[connections[i]].text = ""

        texts_in_image = map(lambda text_in_image: TextInImage(
            re.sub('\s+', ' ', text_in_image.text.replace('\n', '  ')),
            text_in_image.position
        ), texts_in_image)

        return texts_in_image

    def _pre_processing_on_text_img(self, img):
        _, img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
        img = deskew(img)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        return img

    def _post_processing_on_texts(self, texts):
        return self._connect_nearby_name_texts(texts)


class RoomNumberRecognition(TextRecognition):

    def __init__(self, language, kernel_remove_background_shape, kernel_thin_text_shape):
        super(RoomNumberRecognition, self).__init__(language)
        self.kernel_remove_background_shape = kernel_remove_background_shape
        self.kernel_thin_text_shape = kernel_thin_text_shape

    def _pre_processing_on_text_img(self, img):
        _, img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, self.kernel_remove_background_shape))
        img = cv2.dilate(img, cv2.getStructuringElement(cv2.MORPH_RECT, self.kernel_thin_text_shape))
        img = deskew(img)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        return img
