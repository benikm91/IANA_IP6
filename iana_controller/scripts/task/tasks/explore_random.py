import threading
import rospy
import actionlib
import iana_navigation.msg

from task.task import Task


class ExploreRandomTask(Task):

    def __init__(self, msg):
        super(ExploreRandomTask, self).__init__()
        self.explore_random_action = actionlib.SimpleActionClient('/iana/navigation/explore_random', iana_navigation.msg.ExploreRandomAction)
        if not self.explore_random_action.wait_for_server(rospy.Duration(1)):
            rospy.logerr('Failed to connect to /iana/navigation/explore_random')
            self.terminated.set()
        self.goal = iana_navigation.msg.ExploreRandomGoal(until=msg.until)
        self.running = threading.Event()

    @property
    def name(self):
        return "Explore Random Task"

    def update(self, elapsed):
        pass

    def on_start(self):
        rospy.logerr('start random exploring')
        self.running.set()
        self.explore_random_action.send_goal(self.goal, self._goal_reached_callback)

    def on_resume(self):
        rospy.logerr('resume random exploring')
        self.on_start()

    def on_interrupt(self):
        rospy.logerr('interrupt random exploring')
        self.running.clear()
        self.explore_random_action.cancel_goal()

    def on_shutdown(self):
        rospy.logerr('shutdown random exploring')
        self.running.clear()
        self.explore_random_action.cancel_goal()
        self.terminated.set()

    def interruptable_by(self, task):
        return True

    def _goal_reached_callback(self, state, result):
        if self.running.is_set():
            rospy.logerr('random exploring goal reached: terminated set!')
            self.terminated.set()