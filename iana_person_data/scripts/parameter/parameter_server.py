import rospy


class ParameterServer(object):

    @staticmethod
    def database_file():
        return rospy.get_param('~database_file', 'database.db')