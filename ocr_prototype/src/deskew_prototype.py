import cv2
import numpy as np

start = cv2.imread('../res/before_deskew.png', 1)
_, img = cv2.threshold(cv2.cvtColor(start, cv2.COLOR_BGR2GRAY), 0.0, 255.0, cv2.THRESH_OTSU + cv2.THRESH_BINARY)

cv2.imshow('start', start)
cv2.waitKey(0)

a = np.where(img == 0)
coords = np.column_stack([ a[1], a[0] ])
rect = cv2.minAreaRect(coords)
angle = rect[-1]
if (angle < 0):
    angle += 180
box = cv2.boxPoints(rect)
box = np.int0(box)
cv2.drawContours(start, [box], 0, (0, 0, 128), 2)

angle = 90 - angle

print "Found angle: " + str(angle)

cv2.imshow('found angled rectangle', start)
cv2.waitKey(0)

(h, w) = start.shape[:2]
M = cv2.getRotationMatrix2D((w // 2, h // 2), -angle, 1.0)
start = cv2.warpAffine(start, M, (w, h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)

cv2.imshow('rotated', start)
cv2.waitKey(0)

cv2.destroyAllWindows()
