import rospy
import iana_orientation.srv


class MappingController(object):

    def __init__(self):
        super().__init__()
        s = rospy.Service('/iana/orientation/set_mapping_mode', iana_orientation.srv.SetMappingMode, self.handle_set_mapping_mode)

    def handle_set_mapping_mode(self,req):
        # TODO: implement!
        return iana_orientation.srv.SetMappingModeResponse(True)