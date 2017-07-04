import cv2
import numpy as np


# -----------------------
# PARAMS
# -----------------------

# img_path = "212rgb_image_raw.jpg"
img_path = "dlink_zoom_1_5m_2017-06-27_13-28-13.027401.jpg"

morph_kernel_grad_shape = (3, 3)
morph_kernel_close_shape = (9, 1)
threshold_min_text_ratio = 0.5
threshold_min_height = 8
threshold_min_width = 8

# -----------------------# 

def blend_non_transparent(face_img, overlay_img):
    # Let's find a mask covering all the non-black (foreground) pixels
    # NB: We need to do this on grayscale version of the image
    gray_overlay = cv2.cvtColor(overlay_img, cv2.COLOR_BGR2GRAY)
    overlay_mask = cv2.threshold(gray_overlay, 1, 255, cv2.THRESH_BINARY)[1]

    # Let's shrink and blur it a little to make the transitions smoother...
    overlay_mask = cv2.erode(overlay_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    overlay_mask = cv2.blur(overlay_mask, (3, 3))

    # And the inverse mask, that covers all the black (background) pixels
    background_mask = 255 - overlay_mask

    # Turn the masks into three channel, so we can use them as weights
    overlay_mask = cv2.cvtColor(overlay_mask, cv2.COLOR_GRAY2BGR)
    background_mask = cv2.cvtColor(background_mask, cv2.COLOR_GRAY2BGR)

    # Create a masked out face image, and masked out overlay
    # We convert the images to floating point in range 0.0 - 1.0
    face_part = (face_img * (1 / 255.0)) * (background_mask * (1 / 255.0))
    overlay_part = (overlay_img * (1 / 255.0)) * (overlay_mask * (1 / 255.0))

    # And finally just add them together, and rescale it back to an 8bit integer image
    return np.uint8(cv2.addWeighted(face_part, 255.0, overlay_part, 255.0, 0.0))


# removes pixels in image that are between the range of
# [lower_val,upper_val]
def remove_gray(img, lower_val, upper_val):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([0, 0, lower_val])
    upper_bound = np.array([255, 255, upper_val])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    mask = 255 - mask
    res = cv2.bitwise_and(img, img, mask=mask)
    bg = np.ndarray(img.shape)
    bg.fill(255)
    return blend_non_transparent(bg, res)

large = cv2.imread(img_path)
cv2.imshow("large", large)
cv2.waitKey(0)

# large = remove_gray(large, 120, 255)
# cv2.imshow("remove_gray", large)
# cv2.waitKey(0)

# rgb = cv2.pyrDown(large)
rgb = large

gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
cv2.imshow("gray", gray)
cv2.waitKey(0)

morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_grad_shape)
grad = cv2.morphologyEx(gray, cv2.MORPH_GRADIENT, morph_kernel)
cv2.imshow("grad", grad)
cv2.waitKey(0)

_, bw = cv2.threshold(grad, 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)
cv2.imshow("bw", bw)
cv2.waitKey(0)

morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, morph_kernel_close_shape)
connected = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, morph_kernel)
cv2.imshow("connceted", connected)
cv2.waitKey(0)

mask = np.zeros(bw.shape)
contours, hierarchy = cv2.findContours(connected, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
hierarchy = hierarchy[0]  # why????

idx = 0
while idx >= 0:
    rect = cv2.boundingRect(contours[idx])
    mask_roi = mask[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]]
    cv2.drawContours(mask, contours, idx, (255, 255, 255), cv2.cv.CV_FILLED)
    r = (float(cv2.countNonZero(mask_roi)) / mask_roi.size) if mask_roi.size > 0 else 0
    print("r = {}, rect = {}".format(r, rect))
    if r > threshold_min_text_ratio and rect[2] > threshold_min_width and rect[3] > threshold_min_height:
        # assume at least 45% of the area is filled if it contains text
        # constraints on region size:
        # these two conditions alone are not very robust. better to use something
        # like the number of significant peaks in a horizontal projection as a third condition
        cv2.rectangle(rgb, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
        print "found text!"
    idx = hierarchy[idx][0]

cv2.imshow("result", rgb)
cv2.waitKey(0)

cv2.destroyAllWindows()
