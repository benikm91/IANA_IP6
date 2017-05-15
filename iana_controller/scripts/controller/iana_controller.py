from taskv2.task_receiver import TaskReceiver
from taskv2.task_system import TaskSystem


class IanaController(object):
    def __init__(self):
        self.task_system = TaskSystem()
        self.task_system.run()
        self.task_receiver = TaskReceiver(self.task_system)
