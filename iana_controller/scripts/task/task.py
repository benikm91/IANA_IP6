import threading


class Task(object):

    def __init__(self):
        super(Task, self).__init__()
        self.terminate = threading.Event()

    def finished(self):
        self.terminate.set()

    def run(self):
        pass

    def interrupt(self):
        pass

    def resume(self):
        pass

    def shutdown(self):
        pass