import rospy
import actionlib
import iana_navigation.msg

from task.task import Task


class ExploreTask(Task):

    def __init__(self, until):
        super(ExploreTask, self).__init__()
        self.explore_action = actionlib.SimpleActionClient('/iana/navigation/explore', iana_navigation.msg.ExploreAction)
        self.explore_action.wait_for_server()
        self.goal = iana_navigation.msg.ExploreGoal(until=until)

    def start(self):
        self.explore_action.send_goal(self.goal, self._goal_reached_callback)

    def resume(self):
        self.start()

    def interrupt(self):
        self.explore_action.cancel_goal()

    def shutdown(self):
        self.explore_action.cancel_goal()

    def interruptable_by(self, task):
        return False

    def _goal_reached_callback(self, state, result):
        self.terminated.set()

