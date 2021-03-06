from task.task_receiver import TaskReceiver
from task.task_system import TaskSystem


class IanaController(object):
    def __init__(self):
        self.task_system = TaskSystem()
        self.task_system.run()
        self.task_receiver = TaskReceiver(self.task_system)
