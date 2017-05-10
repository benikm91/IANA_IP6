from person_detection.grouping.MemoryCounter import MemoryCounter


class FaceGrouper:

    def __init__(self, known_threshold, unknown_threshold, memory_span_known, memory_span_unknown):
        self.memory_known = MemoryCounter(memory_span_known, known_threshold)
        self.memory_unknown = MemoryCounter(memory_span_unknown, unknown_threshold)

    def known_threshold_reached(self, person_id):
        return self.memory_known.min_reached(person_id)

    def update_known(self, person_id, confidence, face_vector, record_timestamp):
        self.memory_known.remove_old()
        self.memory_known.update(person_id, (confidence, face_vector), record_timestamp)

    def known_reset(self, person_id):
        self.memory_known.pop(person_id)

    def unknown_threshold_reached(self, unknown_person_id):
        return self.memory_unknown.min_reached(unknown_person_id)

    def update_unknown(self, unknown_person_id, face_vector, record_timestamp):
        self.memory_unknown.remove_old()
        self.memory_unknown.update(unknown_person_id, face_vector, record_timestamp)

    def unknown_reset(self, unknown_person_id):
        self.memory_unknown.pop(unknown_person_id)