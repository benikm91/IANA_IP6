import os

from iana_person_detection.msg import KnownPersonEntered, KnownPersonLeft, UnknownPersonEntered, UnknownPersonLeft


class KnownPersonEnteredBehavior(object):
    def known_person_entered(self, iana, msg):
        """
        :param iana
        :type iana : IanaController
        :param msg:
        :type msg KnownPersonEntered
        :return:
        """
        pass


class GreetKnownPerson(KnownPersonEnteredBehavior):
    def known_person_entered(self, iana, msg):
        message = "Hello " + msg.person_id
        os.system("say %(message)s" % locals())