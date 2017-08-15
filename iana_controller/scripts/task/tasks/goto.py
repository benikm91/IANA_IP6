import threading
import rospy
import actionlib
import iana_navigation.msg

from task.task import Task


class GoToTask(Task):

    def __init__(self, msg):
        super(GoToTask, self).__init__()
        self.go_to_action = actionlib.SimpleActionClient('iana/navigation/go_to', iana_navigation.msg.GoToAction)
        if not self.go_to_action.wait_for_server(rospy.Duration(1)):
            rospy.logerr('Failed to connect to /iana/navigation/go_to')
            self.terminated.set()
        self.goal = iana_navigation.msg.GoToGoal(target_pose=msg.target_pose)
        self.target_pose = msg.target_pose
        self.running = threading.Event()

    @property
    def name(self):
        return "Go to {} {} Task".format(self.target_pose.pose.position.x, self.target_pose.pose.position.y)

    def update(self, elapsed):
        pass

    def on_start(self):
        self.running.set()
        self.go_to_action.send_goal(self.goal, self._goal_reached_callback)
        rospy.logerr('Goto started!')

    def on_resume(self):
        self.on_start()

    def on_interrupt(self):
        self.running.clear()
        self.go_to_action.cancel_goal()

    def on_shutdown(self):
        self.running.clear()
        self.go_to_action.cancel_goal()
        self.terminated.set()

    def interruptable_by(self, task):
        return True

    def _goal_reached_callback(self, state, result):
        if self.running.is_set():
            self.terminated.set()
