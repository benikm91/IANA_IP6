import rospy
import actionlib
import iana_navigation.msg

from taskv2.task import Task


class ExploreTask(Task):

    def __init__(self, until):
        super(ExploreTask, self).__init__()
        self.explore_action = actionlib.SimpleActionClient('/iana/navigation/explore', iana_navigation.msg.ExploreAction)
        if not self.explore_action.wait_for_server(rospy.Duration(1)):
            rospy.logerr('Failed to connect to /iana/navigation/explore')
            self.terminated.set()
        self.goal = iana_navigation.msg.ExploreGoal(until=until)

    def update(self, elapsed):
        pass

    def on_start(self):
        self.explore_action.send_goal(self.goal, self._goal_reached_callback)

    def on_resume(self):
        self.on_start()

    def on_interrupt(self):
        self.explore_action.cancel_goal()

    def on_shutdown(self):
        self.explore_action.cancel_goal()

    def interruptable_by(self, task):
        return True

    def _goal_reached_callback(self, state, result):
        self.terminated.set()
