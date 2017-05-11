import threading
from Queue import LifoQueue

import rospy

from task import Task
from task_context import TaskContext
from task_list import TaskList


class TaskSystem(object):
    def __init__(self):
        super(TaskSystem, self).__init__()
        self.pending_tasks = TaskList()
        self.interrupted_tasks = LifoQueue()
        self.current_task = None
        self.process_list_thread = threading.Thread(target=self._process_list)
        self.process_list_thread.daemon = True
        self.mutex = threading.Lock()
        self.terminated = threading.Event()
        self.running = threading.Event()

    def submit(self, task, try_push_in=True):
        assert isinstance(task, Task), 'only tasks of type "Task" can be submitted to TaskSystem'
        task = TaskContext(task)
        rospy.loginfo('submit entered')
        with self.mutex:
            if self.current_task is not None and self.current_task.interruptable_by(task):
                rospy.loginfo('interrupt current task!')
                running_task = self.current_task
                self.interrupted_tasks.put(running_task)
                self.current_task = task
                running_task.interrupt()
            else:
                rospy.loginfo('enque task!')
                self.pending_tasks.enqueue(task, True)
                rospy.loginfo('enqued!')

    def run(self):
        self.process_list_thread.start()
        self.running.set()

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
                rospy.loginfo('try get current_task!')
                while self.current_task is not None and self.current_task.terminated.is_set:
                    if not self.interrupted_tasks.empty():
                        self.current_task = self.interrupted_tasks.get()
                rospy.loginfo('current_task is : {}'.format(self.current_task))
            if self.current_task is None:
                rospy.loginfo('get current_task from queue')
                self.current_task = self.pending_tasks.get()
            rospy.loginfo('current_task is : {}'.format(self.current_task))
            rospy.loginfo('wait for run signal')
            self.running.wait()
            rospy.loginfo('now running -> start current_task')
            self.current_task.start()
            self.current_task.wait_until_stopped()