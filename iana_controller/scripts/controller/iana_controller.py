import rospy

from iana_person_detection.msg import UnknownPersonEntered

from interaction.state import GreetingsState


class IanaController(object):

    def __init__(self):
        self.interaction_state = GreetingsState()
        self._init_listen()
        rospy.init_node('iana_controller', anonymous=True)

    def _init_listen(self):
        rospy.Subscriber("/iana/person_detection/unknown/entered",
                         UnknownPersonEntered,
                         lambda msg: self.interaction_state.unknown_person_entered(self, msg),
                         queue_size=10)
