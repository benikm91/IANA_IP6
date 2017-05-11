import threading
from Queue import LifoQueue, Queue

import rospy

from task import Task
from task_context import TaskContext
from task_list import TaskList


class TaskSystem(object):
    def __init__(self):
        super(TaskSystem, self).__init__()

        self.pending_tasks = TaskList()
        self.interrupted_tasks = LifoQueue()
        self.pushed_in_tasks = Queue()

        self.current_task = None
        self.process_list_thread = threading.Thread(target=self._process_list)
        self.process_list_thread.daemon = True

        self.mutex = threading.Lock()
        self.terminated = threading.Event()
        self.running = threading.Event()
        self.tasks_available = threading.Condition(self.mutex)

    def submit(self, task, try_push_in=True):
        assert isinstance(task, Task), 'only tasks of type "Task" can be submitted to TaskSystem'
        task = TaskContext(task)
        rospy.loginfo('New task: {}'.format(task))
        with self.tasks_available:
            # Try to interrupt currently running task
            if try_push_in and self.current_task is not None and not self.current_task.terminated.is_set() and self.current_task.interruptable_by(task):
                rospy.loginfo('Interrupt task: {}'.format(self.current_task))
                self.interrupted_tasks.put(self.current_task)
                self.pushed_in_tasks.put(task)
                self.current_task.interrupt()
                self.tasks_available.notify()
            # No interruptable currently running task found: enqueue task
            else:
                self.pending_tasks.enqueue(task, try_push_in)
                self.tasks_available.notify()

    def run(self):
        rospy.loginfo('Start task system')
        with self.tasks_available:
            self.process_list_thread.start()
            self.running.set()

    def resume(self):
        rospy.loginfo('Resume task system')
        with self.tasks_available:
            self.running.set()

    def interrupt(self):
        rospy.loginfo('Interrupt task system')
        with self.tasks_available:
            if self.current_task is not None:
                rospy.loginfo('Interrupt task: {}'.format(self.current_task))
                self.interrupted_tasks.put(self.current_task)
                self.current_task.interrupt()
                self.tasks_available.notify()
        self.running.clear()

    def shutdown(self):
        rospy.loginfo('Shut down task system')
        with self.tasks_available:
            if self.current_task is not None:
                self.current_task.shutdown()
            self.terminated.set()

    def _has_next_task(self):
        return not self.pushed_in_tasks.empty() or not self.interrupted_tasks.empty() or not self.pending_tasks.empty()

    def _get_next_task(self):
        if not self.pushed_in_tasks.empty():
            return self.pushed_in_tasks.get()
        elif not self.interrupted_tasks.empty():
            return self.interrupted_tasks.get()
        elif not self.pending_tasks.empty():
            return self.pending_tasks.get()
        else:
            raise RuntimeError('No task available! Most likely due to an unhandled race condition.')

    def _process_list(self):
        while not self.terminated.is_set():
            with self.tasks_available:
                while not self._has_next_task():
                    self.tasks_available.wait()
                self.current_task = self._get_next_task()
                self.running.wait()
                if not self.current_task.terminated.is_set():
                    if self.current_task.stopped.is_set():
                        rospy.loginfo('Resume task: {}'.format(self.current_task))
                        self.current_task.resume()
                    else:
                        rospy.loginfo('Start task: {}'.format(self.current_task))
                    self.current_task.start()
            self.current_task.wait_until_stopped()
