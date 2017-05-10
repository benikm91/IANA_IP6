from iana_person_detection.msg import KnownPersonEntered, KnownPersonLeft, UnknownPersonEntered, UnknownPersonLeft


class UnknownPersonLeftBehavior(object):
    def unknown_person_left(self, iana, msg):
        """
        :param iana
        :type iana : Iana
        :param event:
        :type event UnknownPersonLeft
        :return:
        """
        pass