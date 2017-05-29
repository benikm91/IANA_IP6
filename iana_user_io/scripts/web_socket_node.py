#!/usr/bin/env python

import rospy

from action.get_name_server import GetNameActionServer
from ianaio.iana_io import IanaTalker
from ianaio.web_socket import WebSocketIO
from nav_msgs.msg import OccupancyGrid

if __name__ == '__main__':

    import sys

    from twisted.python import log

    log.startLogging(sys.stdout)

    try:

        rospy.init_node('web_socket_node', anonymous=True)

        websocket_io = WebSocketIO(IanaTalker())
        websocket_io.start()

        server = GetNameActionServer('get_name', websocket_io)

        subscriber = rospy.Subscriber("map", OccupancyGrid, lambda map: websocket_io.refresh_map(map.info.width, map.info.height, map.data))

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

