#!/usr/bin/env python

import rospy

from action.get_name_server import GetNameActionServer
from ianaio.iana_io import IanaTalker
from ianaio.web_socket import WebSocketIO
from nav_msgs.msg import OccupancyGrid
from vector.Pose import Pose

if __name__ == '__main__':

    import sys

    from twisted.python import log

    log.startLogging(sys.stdout)

    try:

        rospy.init_node('web_socket_node', anonymous=True)

        websocket_io = WebSocketIO(IanaTalker())
        websocket_io.start()

        server = GetNameActionServer('get_name', websocket_io)

        def map_callback(map):
            pose = Pose.from_ros_msg(map.info.origin)
            websocket_io.refresh_map(map.info.resolution, pose, map.info.width, map.info.height, pose.position.x, pose.position .y, map.data)

        subscriber = rospy.Subscriber("map", OccupancyGrid, map_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

