import cv2
import numpy as np


def detect_letters(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow("img_gray", img_gray)
    cv2.waitKey(0)

    img_sobel = cv2.Sobel(img_gray, cv2.CV_8U, 1, 0, 3, 1, 0, cv2.BORDER_DEFAULT)
    cv2.imshow("img_sobel", img_sobel)
    cv2.waitKey(0)

    _, img_threshold = cv2.threshold(img_sobel, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
    cv2.imshow("img_threshold", img_threshold)
    cv2.waitKey(0)

    element = cv2.getStructuringElement(cv2.MORPH_RECT, (17, 3))
    cv2.morphologyEx(img_threshold, cv2.MORPH_CLOSE, element)
    cv2.imshow("img_threshold", img_threshold)
    cv2.waitKey(0)

    contours = cv2.findContours(img_threshold, 0, 1)
    contours_poly = []
    bound_rect = []
    for contour in contours:
        if len(contour) > 100:
            contour_poly = cv2.approxPolyDP(contour, 3, True)
            contours_poly.append(contour_poly)
            app_rect = cv2.boundingRect(contour_poly)
            if app_rect.width > app_rect.height:
                bound_rect.append(app_rect)
    return bound_rect

# img_path = "212rgb_image_raw.jpg"
img_path = "dlink_zoom_1_5m_2017-06-27_13-28-13.027401.jpg"

large = cv2.imread(img_path)

bbs = detect_letters(large)
for bb in bbs:
    cv2.rectangle(large, bb, (0, 255, 0), 3, 8, 0)

cv2.imshow("Hambbe", large)
cv2.waitKey(0)

cv2.destroyAllWindows()