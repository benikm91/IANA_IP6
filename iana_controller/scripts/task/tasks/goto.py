import rospy
import actionlib
import iana_navigation.msg

from task.task import Task


class GoToTask(Task):

    def __init__(self, target_pose):
        super(GoToTask, self).__init__()
        self.go_to_action = actionlib.SimpleActionClient('iana/navigation/go_to', iana_navigation.msg.GoToAction)
        self.go_to_action.wait_for_server()
        self.goal = iana_navigation.msg.ExploreGoal(target_pose=target_pose)

    def start(self):
        self.go_to_action.send_goal(self.goal, self._goal_reached_callback)

    def resume(self):
        self.start()

    def interrupt(self):
        self.go_to_action.cancel_goal()

    def shutdown(self):
        self.go_to_action.cancel_goal()

    def _goal_reached_callback(self, state, result):
        self.terminated.set()