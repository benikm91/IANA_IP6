

class StateLeaveBehaviour(object):
    def state_leave(self, iana):
        """
        :param iana
        :type iana: IanaController
        :return:
        """
        pass


class InterruptTaskListOnStateLeave(StateLeaveBehaviour):
    def state_leave(self, iana):
        iana.task_list.interrupt()