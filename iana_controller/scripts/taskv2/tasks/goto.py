import rospy
import actionlib
import iana_navigation.msg

from taskv2.task import Task


class GoToTask(Task):

    def __init__(self, target_pose):
        super(GoToTask, self).__init__()
        self.go_to_action = actionlib.SimpleActionClient('iana/navigation/go_to', iana_navigation.msg.GoToAction)
        if not self.go_to_action.wait_for_server(rospy.Duration(1)):
            rospy.logerr('Failed to connect to /iana/navigation/go_to')
            self.terminated.set()
        self.goal = iana_navigation.msg.GoToGoal(target_pose=target_pose)

    def update(self, elapsed):
        pass

    def on_start(self):
        self.go_to_action.send_goal(self.goal, self._goal_reached_callback)

    def on_resume(self):
        self.on_start()

    def on_interrupt(self):
        self.go_to_action.cancel_goal()

    def on_shutdown(self):
        self.go_to_action.cancel_goal()

    def interruptable_by(self, task):
        return True

    def _goal_reached_callback(self, state, result):
        self.terminated.set()