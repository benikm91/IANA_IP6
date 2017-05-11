from person_detection.grouping.MemoryCounter import MemoryCounter


class SessionMemory:

    def __init__(self, memory_span_known=10, memory_span_unknown=10):
        self.memory_known = MemoryCounter(memory_span_known)
        self.memory_unknown = MemoryCounter(memory_span_unknown)

    def known_update(self, person_id, record_timestamp):
        self.memory_known.update(person_id, person_id, record_timestamp)

    def unknown_update(self, unknown_person_id, record_timestamp):
        self.memory_unknown.update(unknown_person_id, unknown_person_id, record_timestamp)

    def known_contains(self, person_id):
        return person_id in self.memory_known.memory.keys()

    def known_remove_old(self):
        return self.memory_known.remove_old()

    def unknown_remove_old(self):
        return self.memory_unknown.remove_old()

    def unknown_contains(self, unknown_person_id):
        return unknown_person_id in self.memory_unknown.memory.keys()