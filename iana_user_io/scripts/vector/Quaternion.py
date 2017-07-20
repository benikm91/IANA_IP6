from Vector3 import Vector3


class Quaternion(object):

    def __init__(self, q, w):
        self.q = q
        self.w = w

    @classmethod
    def from_ros_msg(cls, msg):
        """
        :param msg:
        :type msg: geometry_msgs.msg.Quaternion
        :return: 
        """
        return cls(Vector3(msg.x, msg.y, msg.z), msg.w)

    def __str__(self):
        return "Quaternion({0}, {1})".format(self.q, self.w)

    def __eq__(self, other):
        def approx_eq(f, s, threshold=0.1):
            return abs(f - s) < threshold
        if other is None:
            return False
        if not isinstance(other, Quaternion):
            return False
        return self.q == other.q and approx_eq(self.w, other.w)