import datetime
import cv2
import numpy as np

path = "recordings/"
recording_name = "dlink"
recording_info = "2m"

cap = cv2.VideoCapture("http://admin:@192.168.0.20:80/video1.mjpg")
# cap.set(3, 4416)
# cap.set(4, 1242)

frame = None
while frame is None:
    print "wait for frame..."
    ret, frame = cap.read()

# cv2.imshow('frame', frame)
# cv2.waitKey(1000)
cv2.imwrite('{}_{}_{}_{}.jpg'.format(path, recording_name, recording_info, str(datetime.datetime.now()).replace(" ", "_")), frame)

cap.release()
