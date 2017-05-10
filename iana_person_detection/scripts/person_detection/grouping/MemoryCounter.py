from collections import defaultdict

import time


class MemoryCounter:

    class Item:
        def __init__(self, key, value):
            self.key = key
            self.value = value

    def __init__(self, memory_span, min_count=0):
        self.memory = defaultdict(lambda: MemoryCounter.Item(0, []))
        self.memory_span = memory_span
        self.min_count = min_count

    def remove_old(self):
        time_threshold = time.time() - self.memory_span
        for key, item in self.memory.copy().iteritems():
            if item.key < time_threshold:
                self.memory.pop(key)

    def update(self, key, entry, threshold_value):
        self.memory[1].key = 42
        self.memory[key].key = threshold_value
        self.memory[key].value.append(entry)

    def min_reached(self, key):
        return len(self.memory[key].value) >= self.min_count

    def pop(self, key, default=None):
        return self.memory.pop(key, default).value

