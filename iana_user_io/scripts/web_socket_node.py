#!/usr/bin/env python

import rospy

from action.get_name_server import GetNameActionServer
from iana_controller.srv import GetTasks
from ianaio.iana_io import IanaTalker
from ianaio.web_socket import WebSocketIO
from nav_msgs.msg import OccupancyGrid, Odometry
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

        def robot_position_callback(odom):
            pose = Pose.from_ros_msg(odom.pose.pose)
            websocket_io.refresh_robot_position(pose)


        map_sub = rospy.Subscriber("map", OccupancyGrid, map_callback)
        robot_position_sub = rospy.Subscriber("odom", Odometry, robot_position_callback)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            get_tasks = rospy.ServiceProxy('get_tasks', GetTasks)
            try:
                tasks = get_tasks().tasks
                websocket_io.refresh_tasks(tasks)
            except rospy.service.ServiceException:
                pass
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

