import threading

from task.task import Task


def event_relay(e1, *e2):
    e1.wait()
    for e in e2:
        e.set()


class TaskContext(Task):
    def __init__(self, task):
        super(TaskContext, self).__init__()
        self.task = task
        self.mutex = threading.Lock()
        self.running = threading.Event()
        self.stopped = threading.Event()
        self.task_state_observer = threading.Thread(target=event_relay, args=(self.terminated, self.stopped, self.terminated))

    def start(self):
        with self.mutex:
            if self.running.is_set():
                raise ValueError("Cannot start a Task that is already running")
            if self.stopped.is_set():
                raise ValueError("Cannot start a stopped Task, use resume instead")
            if self.terminated.is_set():
                raise ValueError("Cannot start a Task that has already terminated")
            self.running.set()
            self.task.start()

    def resume(self):
        with self.mutex:
            if self.running.is_set():
                raise ValueError("Cannot resume a Task that is already running")
            if self.terminated.is_set():
                raise ValueError("Cannot resume a Task that has already terminated")
            self.stopped.clear()
            self.running.set()
            self.task.resume()

    def interrupt(self):
        with self.mutex:
            if not self.running.is_set():
                raise ValueError("Cannot interrupt a Task that is not running")
            if self.terminated.is_set():
                raise ValueError("Cannot interrupt a Task that has already terminated")
            self.task.interrupt()
            self.running.clear()
            self.stopped.set()

    def shutdown(self):
        with self.mutex:
            if not self.running.is_set():
                raise ValueError("Cannot shutdown a Task that is not running")
            if self.terminated.is_set():
                raise ValueError("Cannot shutdown a Task that has already terminated")
            self.task.shutdown()
            self.running.clear()
            self.stopped.set()
            self.terminated.set()

    def interruptable_by(self, task):
        with self.mutex:
            return self.task.interruptable_by(task)

    def wait_until_terminated(self, timeout=None):
        self.terminated.wait(timeout)

    def wait_until_stopped(self, timeout=None):
        self.stopped.wait(timeout)

