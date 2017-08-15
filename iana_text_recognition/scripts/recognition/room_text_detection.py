import cv2
import numpy as np


class RoomTextDetection(object):

    def _smurge_text_detection(self, rgb, morph_kernel_remove_background_shape, morph_kernel_grad_shape, morph_kernel_close_shape,
                       threshold_min_text_ratio, threshold_min_height, threshold_min_width):

        def binary_image(img):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, binary_image = cv2.threshold(gray, 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
            return binary_image

        def remove_background(img):
            if morph_kernel_remove_background_shape is None:
                return img
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, morph_kernel_remove_background_shape)
            return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

        def gradient_image(img):
            if morph_kernel_grad_shape is None:
                return img
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_grad_shape)
            return cv2.morphologyEx(img, cv2.MORPH_GRADIENT, kernel)

        def smudge_edges(img):
            if morph_kernel_close_shape is None:
                return img
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, morph_kernel_close_shape)
            return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

        def find_texts(img):
            mask = np.zeros(img.shape)
            _, contours, hierarchy = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
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

        return find_texts(smudge_edges(gradient_image(remove_background(binary_image(rgb)))))

    def text_detection(self, rgb):
        pass


class RoomNameDetection(RoomTextDetection):

    def __init__(self, kernel_grad_shape, kernel_close_shape, threshold_min_text_ratio, threshold_min_height, threshold_min_width):
        self.kernel_grad_shape = kernel_grad_shape
        self.kernel_close_shape = kernel_close_shape
        self.threshold_min_text_ratio = threshold_min_text_ratio
        self.threshold_min_height = threshold_min_height
        self.threshold_min_width = threshold_min_width

    def text_detection(self, rgb):
        return self._smurge_text_detection(rgb, None, self.kernel_grad_shape, self.kernel_close_shape,
            self.threshold_min_text_ratio, self.threshold_min_height, self.threshold_min_width)


class RoomNumberDetection(RoomTextDetection):

    def __init__(self, kernel_remove_background_shape, kernel_grad_shape, kernel_close_shape, threshold_min_text_ratio, threshold_min_height, threshold_min_width):
        self.kernel_remove_background_shape = kernel_remove_background_shape
        self.kernel_grad_shape = kernel_grad_shape
        self.kernel_close_shape = kernel_close_shape
        self.threshold_min_text_ratio = threshold_min_text_ratio
        self.threshold_min_height = threshold_min_height
        self.threshold_min_width = threshold_min_width

    def text_detection(self, rgb):
        return self._smurge_text_detection(rgb, self.kernel_remove_background_shape, self.kernel_grad_shape, self.kernel_close_shape,
            self.threshold_min_text_ratio, self.threshold_min_height, self.threshold_min_width)

