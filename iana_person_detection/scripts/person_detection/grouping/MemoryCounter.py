from collections import defaultdict

import time

from multiprocessing import Lock


class MemoryCounter:

    class Item:
        def __init__(self, key, value):
            self.key = key
            self.value = value

    def __init__(self, memory_span, min_count=0):
        self.memory = defaultdict(lambda: MemoryCounter.Item(0, []))
        self.memory_span = memory_span
        self.min_count = min_count
        self.lock = Lock()

    def remove_old(self):
        with self.lock:
            time_threshold = time.time() - self.memory_span
            result = []
            for key, item in self.memory.copy().iteritems():
                if item.key < time_threshold:
                    result.append((key, self.memory.pop(key)))
            return result

    def update(self, key, entry, threshold_value):
        with self.lock:
            self.memory[key].key = threshold_value
            self.memory[key].value.append(entry)

    def min_reached(self, key):
        with self.lock:
            return len(self.memory[key].value) >= self.min_count

    def pop(self, key, default=None):
        with self.lock:
            return self.memory.pop(key, default).value

