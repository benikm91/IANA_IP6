import threading


class TaskList(object):

    def __init__(self):
        super(TaskList, self).__init__()
        self.tasks = list()
        self.mutex = threading.Lock()
        self.not_empty = threading.Condition(self.mutex)

    def get(self):
        with self.not_empty:
            while len(self.tasks) == 0:
                self.not_empty.wait()
            task = self.tasks.pop(0)
        return task

    def enqueue(self, task, try_push_in=True):
        with self.not_empty:
            if try_push_in:
                push_in_successful = False
                i = 0
                while not push_in_successful and i < len(self.tasks):
                    if self.tasks[i].interruptable_by(task):
                        self.tasks.insert(i, task)
                        push_in_successful = True
                    i += 1
                if not push_in_successful:
                    self.tasks.append(task)
            else:
                self.tasks.append(task)
            self.not_empty.notify()

    def empty(self):
        self.mutex.acquire()
        is_empty = len(self.tasks) == 0
        self.mutex.release()
        return is_empty
