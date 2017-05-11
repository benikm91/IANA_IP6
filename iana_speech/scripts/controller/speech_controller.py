import os

import actionlib
from iana_speech.msg import SayAction


class SpeechController(object):

    def __init__(self):
        self.explore_action = actionlib.SimpleActionServer('/iana/speech/say',
                                                           SayAction, execute_cb=self.say,
                                                           auto_start=False)
        self.explore_action.start()

    def say(self, msg):
        message = msg.message
        os.system("say %(message)s" % locals())
