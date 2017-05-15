import actionlib
from iana_speech.msg import SayAction, SayGoal

from taskv2.task import Task


class GreetUnknownPersonTask(Task):

    def __init__(self, msg):
        super(GreetUnknownPersonTask, self).__init__()
        self.say_action = actionlib.SimpleActionClient('/iana/speech/say', SayAction)
        self.say_action.wait_for_server()
        self.goal = SayGoal("Oh hello there")

    def update(self, elapsed):
        pass

    def on_start(self):
        self.say_action.send_goal(self.goal, self._goal_reached_callback)

    def on_resume(self):
        self.on_start()

    def on_interrupt(self):
        self.say_action.cancel_goal()

    def on_shutdown(self):
        self.say_action.cancel_goal()

    def interruptable_by(self, task):
        return super(GreetUnknownPersonTask, self).interruptable_by(task)

    def _goal_reached_callback(self, state, result):
        self.terminated.set()

