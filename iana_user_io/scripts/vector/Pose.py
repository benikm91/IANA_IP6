from Vector3 import Vector3
from Quaternion import Quaternion


class Pose(object):

    def __init__(self, position, orientaiton):
        """
        :param position: 
        :type position: Vector3
        :param orientaiton: 
        :type orientaiton: Quaternion
        """
        self.position = position
        self.orientation = orientaiton

    @classmethod
    def from_ros_msg(cls, msg):
        """
        :param msg:
        :type msg: geometry_msgs.msg.Pose
        :return: 
        """
        return cls(Vector3.from_ros_msg(msg.position), Quaternion.from_ros_msg(msg.orientation))

    def __str__(self):
        return "Pose({0}, {1})".format(self.position, self.orientation)