import cv2
import numpy as np


def cutout_rectangles(img, rects):
    result = list()
    for rect in rects:
        result.append(img[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]])
    return result


def text_detection(rgb, debug = False):
    def show_debug_img(title, image):
        if debug:
            cv2.imshow(title, image)
            cv2.waitKey(0)

    morph_kernel_grad_shape = (3, 3)
    morph_kernel_close_shape = (9, 1)
    threshold_min_text_ratio = 0.5      # How much percent of the pixels are foreground
    threshold_min_height = 8            # Min height of bounding box
    threshold_min_width = 8             # Min width of bounding box

    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

    show_debug_img("gray", gray)

    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_grad_shape)
    grad = cv2.morphologyEx(gray, cv2.MORPH_GRADIENT, morph_kernel)

    show_debug_img("grad", grad)

    _, bw = cv2.threshold(grad, 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)

    show_debug_img("bw", bw)

    show_debug_img("bw", bw)

    morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, morph_kernel_close_shape)
    connected = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, morph_kernel)

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

    return result

if __name__ == "__main__":
    # img_path = "res/dlink_zoom_1_5m_2017-06-27.jpg"
    img_path = "res/dlink_zoom_color_2m.jpg"
    large = cv2.imread(img_path)
    cv2.imshow("large", large)
    cv2.waitKey(0)
    rects = text_detection(large, True)
    for rect in rects:
        cv2.rectangle(large, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
    cv2.imshow("final", large)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

