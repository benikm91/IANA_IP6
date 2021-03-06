#!/usr/bin/env python
import subprocess

import rospy
from nav_msgs.msg import OccupancyGrid

counter = 0
base_file_name = ""

def save_map(msg):
    global base_file_name
    global counter
    file_path = "{0}_{1}".format(base_file_name, counter)
    command = "rosrun map_server map_saver -f {0}".format(file_path)
    subprocess.call(["/bin/bash",'-c',command])
    counter += 1

if __name__ == '__main__':
    try:
        rospy.init_node('maps_saver')
        base_file_name = rospy.get_param('~file')
        rospy.Subscriber("/map", OccupancyGrid, save_map)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
