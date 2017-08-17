import threading
import rospy
import actionlib
import iana_navigation.msg

from task.task import Task


class ExploreTask(Task):

    def __init__(self, msg):
        super(ExploreTask, self).__init__()
        self.explore_action = actionlib.SimpleActionClient('/iana/navigation/explore', iana_navigation.msg.ExploreAction)
        if not self.explore_action.wait_for_server(rospy.Duration(1)):
            rospy.logerr('Failed to connect to /iana/navigation/explore')
            self.terminated.set()
        self.goal = iana_navigation.msg.ExploreGoal(until=msg.until)
        self.running = threading.Event()

    @property
    def name(self):
        return "Explore Task"

    def update(self, elapsed):
        pass

    def on_start(self):
        rospy.loginfo('Start exploring')
        self.running.set()
        self.explore_action.send_goal(self.goal, self._goal_reached_callback)

    def on_resume(self):
        rospy.loginfo('resume exploring')
        self.on_start()

    def on_interrupt(self):
        rospy.loginfo('interrupt exploring')
        self.running.clear()
        self.explore_action.cancel_goal()

    def on_shutdown(self):
        rospy.loginfo('shutdown exploring')
        self.running.clear()
        self.explore_action.cancel_goal()
        self.terminated.set()

    def interruptable_by(self, task):
        return True

    def _goal_reached_callback(self, state, result):
        if self.running.is_set():
            rospy.loginfo('exploring goal reached: terminated set!')
            self.terminated.set()

