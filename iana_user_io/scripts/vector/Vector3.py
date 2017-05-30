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