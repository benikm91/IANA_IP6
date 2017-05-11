import threading


class Task(object):

    def __init__(self):
        super(Task, self).__init__()
        self.terminated = threading.Event()

    def start(self):
        raise NotImplementedError()

    def interrupt(self):
        raise NotImplementedError()

    def resume(self):
        raise NotImplementedError()

    def shutdown(self):
        raise NotImplementedError()

    def interruptable_by(self, task):
        return False
