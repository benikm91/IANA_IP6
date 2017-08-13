import threading


class Task(object):

    def __init__(self):
        super(Task, self).__init__()
        self.terminated = threading.Event()

    @property
    def name(self):
        raise NotImplementedError()

    def update(self, elapsed):
        raise NotImplementedError()

    def on_start(self):
        raise NotImplementedError()

    def on_resume(self):
        raise NotImplementedError()

    def on_interrupt(self):
        raise NotImplementedError()

    def on_shutdown(self):
        raise NotImplementedError()

    def interruptable_by(self, task):
        return False
