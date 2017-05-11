import threading


class TaskList(object):

    def __init__(self):
        super(TaskList, self).__init__()
        self.tasks = list()
        self.mutex = threading.Lock()
        self.not_empty = threading.Condition(self.mutex)

    def get(self):
        self.not_empty.aquire()
        while len(self.tasks) == 0:
            self.not_empty.wait()
        task = self.tasks.pop(0)
        self.not_empty.release()
        return task

    def enqueue(self, task, try_push_in=True):
        self.not_empty.aquire()
        if try_push_in:
            for i in range(len(self.tasks)):
                if self.tasks[0].interruptable_by(task):
                    self.tasks.insert(i, task)
        else:
            self.tasks.append(task)
        self.not_empty.notify_all()
        self.not_empty.release()
