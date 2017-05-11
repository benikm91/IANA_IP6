import threading
from Queue import Queue, LifoQueue
from collections import deque

from task.task import Task
from task.task_context import TaskContext
from task.task_list import TaskList


class TaskSystem(object):
    def __init__(self):
        super(TaskSystem, self).__init__()
        self.queue = TaskList()
        self.interrupted_tasks = LifoQueue()
        self.current_task = None
        self.process_list_thread = threading.Thread(target=self._process_list)
        self.mutex = threading.Lock()
        self.terminated = threading.Event()
        self.running = threading.Event()

    def submit(self, task, try_push_in=True):
        assert isinstance(task, Task), 'only tasks of type "Task" can be submitted to TaskSystem'
        task = TaskContext(task)
        with self.mutex:
            if self.current_task is not None and self.current_task.interruptable_by(task):
                running_task = self.current_task
                self.interrupted_tasks.put(running_task)
                self.current_task = task
                running_task.interrupt()
            else:
                self.queue.enqueue(task, True)

    def run(self):
        self.process_list_thread.start()

    def resume(self):
        with self.mutex:
            if self.current_task is not None:
                self.current_task.resume()
            self.running.set()

    def interrupt(self):
        with self.mutex:
            if self.current_task is not None:
                self.current_task.interrupt()
            self.running.clear()

    def shutdown(self):
        with self.mutex:
            if self.current_task is not None:
                self.current_task.shutdown()
            self.terminated.set()

    def _process_list(self):
        while not self.terminated.is_set():
            with self.mutex:
                while self.current_task.terminated.is_set:
                    if not self.interrupted_tasks.empty():
                        self.current_task = self.interrupted_tasks.get()
                    else:
                        self.current_task = self.queue.get()
            self.running.wait()
            self.current_task.start()
            self.current_task.wait_until_stopped()