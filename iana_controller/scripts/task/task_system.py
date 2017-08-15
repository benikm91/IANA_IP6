import threading
from Queue import LifoQueue, Queue

import copy
import rospy
import time

from task import Task
from task_list import TaskList


class TaskSystem(object):
    def __init__(self):
        super(TaskSystem, self).__init__()

        self.pending_tasks = TaskList()
        self.interrupted_tasks = LifoQueue()
        self.pushed_in_tasks = Queue()

        self.process_list_thread = threading.Thread(target=self._process_list)
        self.process_list_thread.daemon = True

        self.mutex = threading.Lock()
        self.tasks_available = threading.Condition(self.mutex)

        # thread safe event flags
        self.running = threading.Event()
        self.interrupted = threading.Event()
        self.terminated = threading.Event()

        # owned by process list thread:
        self.current_task = None
        self.update_interval = 0.01666667  # 60 hz

    def submit(self, task, try_push_in=True):
        assert isinstance(task, Task), 'only tasks of type "Task" can be submitted to TaskSystem'
        if self.terminated.is_set():
            raise RuntimeError('Invalid operation: can\'t submit new task after shutdown'.format())
        rospy.loginfo('New task: {}'.format(task))
        with self.tasks_available:
            if try_push_in:
                self.pushed_in_tasks.put(task)
            else:
                self.pending_tasks.enqueue(task, False)
            self.tasks_available.notify()

    def run(self):
        rospy.loginfo('Start task system')
        if self.terminated.is_set():
            raise RuntimeError('Invalid operation: can\'t start after shutdown'.format())
        self.process_list_thread.start()
        self.running.set()

    def resume(self):
        rospy.loginfo('Resume task system')
        if self.terminated.is_set():
            raise RuntimeError('Invalid operation: can\'t resume after shutdown'.format())
        self.interrupted.clear()
        self.running.set()

    def interrupt(self):
        rospy.loginfo('Interrupt task system')
        if self.terminated.is_set():
            raise RuntimeError('Invalid operation: can\'t interrupt after shutdown'.format())
        self.running.clear()
        self.interrupted.set()

    def shutdown(self):
        rospy.loginfo('Shut down task system')
        if self.terminated.is_set():
            raise RuntimeError('Invalid operation: already shutdown'.format())
        self.running.clear()
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
        self.running.wait()
        while not self.terminated.is_set():
            start = time.time()

            # handle interrupt and wait for running flag
            if self.interrupted.is_set():
                if self.current_task is not None:
                    rospy.loginfo('Interrupt task: {}'.format(self.current_task))
                    self.current_task.on_interrupt()
                self.running.wait()
                if self.current_task is not None:
                    self.current_task.on_resume()
                    rospy.loginfo('Resume task: {}'.format(self.current_task))
                start = time.time()

            # if there is a current_task: check if terminated
            if self.current_task is not None and self.current_task.terminated.is_set():
                rospy.loginfo('Current task terminated: {}'.format(self.current_task))
                self.current_task = None

            with self.tasks_available:
                # if there is no current_task: pull or wait for next
                if self.current_task is None:
                    while not self._has_next_task():
                        self.tasks_available.wait()
                    self.current_task = self._get_next_task()
                    self.current_task.on_start()
                    rospy.loginfo('Start task: {}'.format(self.current_task))
                    start = time.time()

                # in contrast, if there is a current_task: check for pushed_in tasks and handle accordingly
                elif not self.pushed_in_tasks.empty():
                    pushed_in_task = self.pushed_in_tasks.get()
                    if self.current_task.interruptable_by(pushed_in_task):
                        rospy.loginfo('Interrupt task: {}'.format(self.current_task))
                        rospy.loginfo('Push in and start task: {}'.format(pushed_in_task))
                        self.current_task.on_interrupt()
                        self.interrupted_tasks.put(self.current_task)
                        self.current_task = pushed_in_task
                        self.current_task.on_start()
                    else:
                        rospy.loginfo('Enqueue task: {}'.format(pushed_in_task))
                        self.pending_tasks.enqueue(pushed_in_task, True)

            # let current task do an update step
            if self.current_task is not None:
                self.current_task.update(time.time() - start)

            elapsed = (time.time() - start)
            if elapsed > self.update_interval:
                rospy.logwarn('Task update can\'t keep up with update rate: took {} instead of {}.'.format(elapsed, self.update_interval))
            else:
                time.sleep(self.update_interval - elapsed)

        # shutdown nicely :)
        if self.current_task is not None:
            self.current_task.on_shutdown()
        with self.tasks_available:
            while not self.pending_tasks.empty():
                self.pending_tasks.get().on_shutdown()
            while not self.interrupted_tasks.empty():
                self.interrupted_tasks.get().on_shutdown()
            while not self.pushed_in_tasks.empty():
                self.pushed_in_tasks.get().on_shutdown()

        rospy.loginfo('Task system successfully shutdown')

    def get_all_tasks(self):

        result = list()
        if self.current_task is not None:
            result.append(self.current_task)
        for items in [
                self.pushed_in_tasks.queue,
                self.interrupted_tasks.queue,
                self.pending_tasks.tasks,
            ]:
            for item in items:
                result.append(item)
        return result
