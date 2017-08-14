import cv2
import numpy as np


def cutout_rectangles(img, rects):
    result = list()
    for rect in rects:
        result.append(img[rect[1]-5:rect[1]+rect[3]+5, rect[0]-5:rect[0]+rect[2]+5])
    return result


def number_detection(rgb, debug = False):
    def show_debug_img(title, image):
        if debug:
            cv2.imshow(title, image)
            cv2.waitKey(0)

    morph_kernel_grad_shape = (2, 2)
    morph_kernel_close_shape = (9, 1)
    threshold_min_text_ratio = 0.5      # How much percent of the pixels are foreground
    threshold_min_height = 80            # Min height of bounding box
    threshold_min_width = 40             # Min width of bounding box

    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

    show_debug_img("gray", gray)

    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_grad_shape)

    _, bw = cv2.threshold(gray, 130.0, 255.0, cv2.THRESH_BINARY)

    show_debug_img("grad", bw)

    bw = cv2.bitwise_not(bw)

    show_debug_img("bw", bw)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (12, 12))

    show_debug_img("bw1", cv2.erode(bw, kernel))

    bw = cv2.dilate(cv2.erode(bw, kernel), kernel)

    show_debug_img("bw2", bw)

    grad = cv2.morphologyEx(bw, cv2.MORPH_GRADIENT, morph_kernel)

    show_debug_img("bw3", grad)

    _, bw = cv2.threshold(grad, 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)

    show_debug_img("bw", bw)

    # morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, morph_kernel_close_shape)
    # connected = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, morph_kernel)

    connected = bw

    # connected = cv2.bitwise_not(connected)

    show_debug_img("connected", connected)

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
        if debug:
            print("r = {}, rect = {}".format(r, rect))
        if r > threshold_min_text_ratio and rect[2] > threshold_min_width and rect[3] > threshold_min_height:
            """
            assume at least 45% of the area is filled if it contains text
            constraints on region size:
            these two conditions alone are not very robust. better to use something
            like the number of significant peaks in a horizontal projection as a third condition
            """
            result.append(rect)
        idx = hierarchy[idx][0]

    # Combine rectangles to own big rectangle
    left_x = min(map(lambda r: r[0], result))
    up_y = min(map(lambda r: r[1], result))
    right_x = max(map(lambda r: r[0] + r[2], result))
    down_y = max(map(lambda r: r[1] + r[3], result))

    return  [(left_x, up_y, right_x - left_x, down_y - up_y)]

if __name__ == "__main__":
    # img_path = "res/dlink_zoom_1_5m_2017-06-27.jpg"
    # img_path = "res/dlink_zoom_medium_high_color_2m.jpg"
    # img_path = "res/dlink_zoom_medium_color_2m.jpg"
    img_path = "res/dlink_zoom_color_2m.jpg"
    large = cv2.imread(img_path)
    cv2.imshow("large", large)
    cv2.waitKey(0)
    rects = number_detection(large, True)
    for rect in rects:
        cv2.rectangle(large, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
    cv2.imshow("final", large)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
