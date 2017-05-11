import actionlib
import iana_speech.msg

from task.task import Task


class GreetUnknownPersonTask(Task):

    def __init__(self, msg):
        super(GreetUnknownPersonTask, self).__init__()
        self.say_action = actionlib.SimpleActionClient('iana/speech/say', iana_speech.msg.SayAction)
        self.say_action.wait_for_server()
        self.goal = iana_speech.msg.SayGoal("Hallo")

    def start(self):
        self.say_action.send_goal(self.goal, self._goal_reached_callback)

    def resume(self):
        self.start()

    def interrupt(self):
        self.say_action.cancel_goal()

    def shutdown(self):
        self.say_action.cancel_goal()

    def interruptable_by(self, task):
        return super(GreetUnknownPersonTask, self).interruptable_by(task)

    def _goal_reached_callback(self, state, result):
        self.terminated.set()

