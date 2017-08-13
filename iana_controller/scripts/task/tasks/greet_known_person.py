import actionlib
import iana_speech.msg

from task.task import Task


class GreetKnownPersonTask(Task):

    def __init__(self, msg):
        super(GreetKnownPersonTask, self).__init__()
        self.say_action = actionlib.SimpleActionClient('iana/speech/say', iana_speech.msg.SayAction)
        self.say_action.wait_for_server()
        self.person = msg.person

    @property
    def name(self):
        return "Greet known person task"

    def update(self, elapsed):
        pass

    def on_start(self):
        self.say_action.send_goal(iana_speech.msg.SayGoal("Hallo {0}".format(self.person.name)), self._goal_reached_callback)

    def on_resume(self):
        self.on_start()

    def on_interrupt(self):
        self.say_action.cancel_goal()

    def on_shutdown(self):
        self.say_action.cancel_goal()

    def interruptable_by(self, task):
        return super(GreetKnownPersonTask, self).interruptable_by(task)

    def _goal_reached_callback(self, state, result):
        self.terminated.set()

