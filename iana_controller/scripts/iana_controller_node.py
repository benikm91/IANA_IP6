#!/usr/bin/env python

import rospy
from controller.iana_controller import IanaController
from iana_controller.srv import GetTasksResponse, GetTasks

if __name__ == '__main__':

    try:
        rospy.init_node('iana_controller', anonymous=True)
        controller = IanaController()

        def handle_get_tasks(request):
            return GetTasksResponse(tasks=map(lambda x: x.name, controller.task_system.get_all_tasks()))

        get_tasks = rospy.Service('get_tasks', GetTasks, handle_get_tasks)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
