import actionlib
from iana_speech.msg import SayAction, SayGoal

from task.task import Task


class GoodbyeUnknownPersonTask(Task):

    def __init__(self, msg):
        super(GoodbyeUnknownPersonTask, self).__init__()
        self.say_action = actionlib.SimpleActionClient('/iana/speech/say', SayAction)
        self.say_action.wait_for_server()
        self.goal = SayGoal("Bye! Bye!")

    def start(self):
        self.say_action.send_goal(self.goal, self._goal_reached_callback)

    def resume(self):
        self.start()

    def interrupt(self):
        self.say_action.cancel_goal()

    def shutdown(self):
        self.say_action.cancel_goal()

    def interruptable_by(self, task):
        return super(GoodbyeUnknownPersonTask, self).interruptable_by(task)

    def _goal_reached_callback(self, state, result):
        self.terminated.set()

