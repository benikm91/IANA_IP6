import rospy
import iana_orientation.srv


class MappingController(object):

    def __init__(self):
        super(MappingController, self).__init__()
        self.set_mapping_mode_serviceia = rospy.Service('/iana/orientation/set_mapping_mode', iana_orientation.srv.SetMappingMode, self.handle_set_mapping_mode)

    def handle_set_mapping_mode(self,req):
        # TODO: implement!
        return iana_orientation.srv.SetMappingModeResponse(True)