import rospy
import actionlib

import iana_driver.msg
import iana_navigation.msg
import move_base_msgs.msg


class NavigationController(object):

    def __init__(self):
        self.explore_action_server = actionlib.SimpleActionServer('/iana/navigation/explore', iana_navigation.msg.ExploreAction, execute_cb=self.explore, auto_start=False)
        self.go_to_action_server = actionlib.SimpleActionServer('/iana/navigation/go_to', iana_navigation.msg.GoToAction, execute_cb=self.go_to, auto_start=False)

        self.move_base_action = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        if not self.move_base_action.wait_for_server(rospy.Duration(1)):
            rospy.logerr('Failed to connect to /move_base action')

        self.random_driver_publisher = rospy.Publisher('/iana/driver/random', iana_driver.msg.RandomDriverArgs, queue_size=10)
        self.stop_driver_publisher = rospy.Publisher('/iana/driver/stop', iana_driver.msg.NoDriverArgs, queue_size=10)

        self.explore_action_server.start()
        self.go_to_action_server.start()
        rospy.loginfo('navigation controller initialized!')

    def explore(self, goal):
        rospy.loginfo('Explore action received!')
        until = rospy.Time(goal.until.data.secs, goal.until.data.nsecs)

        # start random driver
        self.random_driver_publisher.publish()

        # check for preempted until time's up
        interval = 0.1
        preempted = False
        rospy.loginfo('Explore for {} seconds'.format((until - rospy.get_rostime()).to_sec()))
        while not rospy.get_rostime() >= until and not preempted:
            if self.explore_action_server.is_preempt_requested():
                self.stop_driver_publisher.publish()
                self.explore_action_server.set_preempted()
                preempted = True
            else:
                duration = min((until - rospy.get_rostime()).to_sec(), interval)
                if duration > 0:
                    rospy.sleep(duration)

        if not preempted:
            rospy.loginfo('Explore goal reached!')
            self.explore_action_server.set_succeeded()

    def go_to(self, goal):
        # send target pose to move base
        base_goal = move_base_msgs.msg.MoveBaseActionGoal(target_pose=goal.target_pose)
        self.move_base_action.send_goal(base_goal)

        # check for preempted until goal reached
        interval = 0.1
        preempted = False
        while not self.move_base_action.wait_for_result(interval) and not preempted:
            if self.go_to_action_server.is_preempt_requested():
                self.move_base_action.cancel_goal()
                self.go_to_action_server.set_preempted()
                preempted = True

        base_result = self.move_base_action.get_result()
        result = iana_navigation.msg.GoToResult(base_position=base_result.base_position)

        if not preempted:
            self.go_to_action_server.set_succeeded(result)