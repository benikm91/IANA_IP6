import os

from iana_person_detection.msg import KnownPersonEntered, KnownPersonLeft, UnknownPersonEntered, UnknownPersonLeft


class UnknownPersonEnteredBehavior(object):
    def unknown_person_entered(self, iana, msg):
        """
        :param iana
        :type iana : Iana
        :param event:
        :type event UnknownPersonEntered
        :return:
        """
        pass


class GreetUnknownPerson(UnknownPersonEnteredBehavior):
    def unknown_person_entered(self, iana, event):
        message = "Hello Unknown?"
        os.system("say %(message)s" % locals())