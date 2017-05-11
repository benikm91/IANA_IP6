

class StateEnterBehaviour(object):
    def state_enter(self, iana):
        """
        :param iana
        :type iana: IanaController
        :return:
        """
        pass


class ResumeTaskListOnStateEnter(StateEnterBehaviour):
    def state_enter(self, iana):
        iana.task_list.resume()