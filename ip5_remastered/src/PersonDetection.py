
class PersonDetection(object):

    def __init__(self, face_detection, face_alignment, face_embedder, face_labeler, face_filter, unknown_face_labeler, face_grouper, session_memory):
        """
        :type face_detection: FaceDetection.FaceDetection
        :type face_alignment: FaceAlignment.FaceAlignment
        :type face_embedder: FaceEmbedder.FaceEmbedder
        :type face_labeler: FaceLabeler.FaceLabeler
        :type face_filter: FaceFilter.FaceFilter
        :type unknown_face_labeler: UnknownFaceLabeler.UnknownFaceLabeler
        :type face_grouper: FaceGrouper.FaceGrouper
        :type session_memory: SessionMemory.SessionMemory
        """
        self.face_detection = face_detection
        self.face_alignment = face_alignment
        self.face_embedder = face_embedder
        self.face_labeler = face_labeler
        self.face_filter = face_filter
        self.unknown_face_labeler = unknown_face_labeler
        self.face_grouper = face_grouper
        self.session_memory = session_memory

    def known_person_detected(self, person_id, record_timestamp):
        print "known person detected"
        pass

    def unknown_person_detected(self, unknown_person_id, record_timestamp):
        print "unknown person detected"
        pass

    def detect_person(self, image, record_timestamp):

        def handle_known_face(self, person_id, confidence, face_vector):
            self.face_grouper.update_known(person_id, confidence, face_vector, record_timestamp)
            if self.face_grouper.known_threshold_reached(person_id):
                self.face_grouper.known_reset(person_id)
                if not self.session_memory.known_contains(person_id):
                    self.known_person_detected(person_id, record_timestamp)
                self.session_memory.known_update(person_id, record_timestamp)

        def handle_unknown_face(self, face_vector):
            unknown_person_id = self.unknown_face_labeler.label(face_vector)
            self.face_grouper.update_unknown(unknown_person_id, face_vector, record_timestamp)
            if self.face_grouper.unknown_threshold_reached(unknown_person_id):
                self.face_grouper.unknown_reset(unknown_person_id)
                if not self.session_memory.unknown_contains(unknown_person_id):
                    self.unknown_person_detected(unknown_person_id, record_timestamp)
                self.session_memory.unknown_update(person_id, record_timestamp)

        boundboxes = self.face_detection.detect_faces(image)
        aligned_faces = self.face_alignment.align_faces(image, boundboxes)
        embeddings = self.face_embedder.embed(aligned_faces)
        labeled_faces = self.face_labeler.label(embeddings)

        for person_id, confidence, face_vector in labeled_faces:
            if self.face_filter.is_known(face_vector, confidence):
                handle_known_face(self, person_id, face_vector, confidence)
            elif self.face_filter.is_unknown(face_vector, confidence):
                handle_unknown_face(self, face_vector)