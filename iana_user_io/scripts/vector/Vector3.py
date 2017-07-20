class Vector3(object):

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_ros_msg(cls, msg):
        """
        :param msg:
        :type msg: geometry_msgs.msg.Point
        :return: 
        """
        return cls(msg.x, msg.y, msg.z)

    def __str__(self):
        return "Vector3({0}, {1}, {2})".format(self.x, self.y, self.z)

    def __eq__(self, other):
        def approx_eq(f, s, threshold=0.1):
            return abs(f - s) < threshold
        if other is None:
            return False
        if not isinstance(other, Vector3):
            return False
        return approx_eq(self.x, other.x) and approx_eq(self.y, other.y) and approx_eq(self.z, other.z)