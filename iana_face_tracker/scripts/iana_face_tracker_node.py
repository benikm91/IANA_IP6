#!/usr/bin/env python
import struct
import rospy
import iana_camera_pan_tilt.msg
import iana_person_detection.msg
import std_msgs.msg


class FaceTracker(object):

    def __init__(self, fov, resolution, hold_position_time, pant_tilt_pub) -> None:
        super().__init__()
        self.fov = fov
        self.resolution = resolution
        self.hold_position_time = hold_position_time
        self.pant_tilt_pub = pant_tilt_pub
        self.enabled = False
        self.state = (0, 0)
        self.rotated = False
        self.rotate_back_timer = 0.0

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def on_faces_detected(self, faces):
        if self.enabled:
            position_x = faces[0].x + (faces[0].width / 2.0)
            position_y = faces[0].x + (faces[0].height / 2.0)
            tilt = (position_x / self.resolution[0]) * fov_h
            pan = (position_y / self.resolution[1]) * fov_v
            self.rotated = True
            self.rotate_back_timer = self.hold_position_time
            self.pant_tilt_pub.publish(pan, tilt)

    def on_pan_tilt(self, pan_tilt):
        self.state = (pan_tilt.tilt, pan_tilt.pan)

    def update(self, delta_time):
        if self.rotated:
            if self.rotate_back_timer <= 0.0:
                self.pant_tilt_pub.publish(0, 0)
                self.rotated = False
                self.rotate_back_timer = 0.0
            else:
                self.rotate_back_timer -= delta_time


if __name__ == '__main__':
    try:
        rospy.init_node('iana_face_tracker', anonymous=True)

        fov_h = rospy.get_param("fov_h", 110)  # zed default (degree)
        fov_v = rospy.get_param("fov_v", 45)  # zed default (degree)
        camera_img_width = rospy.get_param("camera_img_width", 1920)  # zed default
        camera_img_height = rospy.get_param("camera_img_height", 1080)  # zed default
        hold_position_time = rospy.get_param("hold_position_time", 3.0)  # seconds

        pant_tilt_pub = rospy.Publisher("/iana/camera/set_pan_tilt", iana_camera_pan_tilt.msg.PanTilt, queue_size=1)

        face_tracker = FaceTracker((fov_h, fov_v), (camera_img_width, camera_img_height), hold_position_time, pant_tilt_pub)

        rospy.Subscriber("/iana/face_tracker/enable", std_msgs.msg.Empty, face_tracker.enable, queue_size=1)
        rospy.Subscriber("/iana/face_tracker/disable", std_msgs.msg.Empty, face_tracker.disable, queue_size=1)
        rospy.Subscriber("/iana/faces_detected", iana_person_detection.msg.FaceBoundingBoxes, face_tracker.on_faces_detected, queue_size=1)
        rospy.Subscriber("/iana/camera/pan_tilt", iana_camera_pan_tilt.msg.PanTilt, face_tracker.on_pan_tilt, queue_size=1)

        rate = rospy.Rate(2)
        last_update = rospy.get_time()
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            face_tracker.update(current_time - last_update)
            last_update = current_time
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
