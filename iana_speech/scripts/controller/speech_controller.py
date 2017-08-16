import os

import rospy
import actionlib
from iana_speech.msg import SayAction


class SpeechController(object):

    def __init__(self):
        self.speech_action = actionlib.SimpleActionServer('/iana/speech/say',
                                                           SayAction,
                                                           execute_cb=self.say)

    def say(self, msg):
        message = msg.message
        rospy.loginfo("IANA says: \"message\"")
        os.system("say %(message)s" % locals())
        self.speech_action.set_succeeded()
