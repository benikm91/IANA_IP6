import threading
from Queue import Queue

from task.task import Task


class TaskSystem(object):
    def __init__(self):
        super(TaskSystem, self).__init__()
        self.queue = Queue()
        self.current_task = None
        self.thread = threading.Thread(target=self._run)
        self.is_shutdown = threading.Event()
        self.is_running = threading.Event()

    def submit(self, task):
        assert isinstance(task, Task), 'only tasks of type Task can be submitted to TaskSystem'
        self.queue.put(task)

    def run(self):
        self.thread.start()

    def _run(self):
        while not self.is_shutdown.is_set():
            self.current_task = self.queue.get()
            self.is_running.wait()
            self.current_task.run()
            self.current_task.terminate.wait()

    def interrupt(self):
        if self.current_task is not None:
            self.current_task.interrupt()
        self.is_running.clear()

    def resume(self):
        if self.current_task is not None:
            self.current_task.resume()
        self.is_running.set()

    def shutdown(self):
        if self.current_task is not None:
            self.current_task.shutdown()
        self.is_shutdown.set()
