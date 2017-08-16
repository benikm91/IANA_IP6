import dlib
import rospy
import time
from std_msgs.msg import Header

import cv2

from iana_person_detection.msg import UnknownPersonEntered, FaceVector, FaceBoundingBoxes, BoundingBox
from cv_bridge import CvBridge

bridge = CvBridge()

class PersonDetection(object):

    def  __init__(self, face_detection, face_alignment, face_embedder, face_labeler, face_filter, unknown_face_labeler, face_grouper, session_memory, faces_detected_publisher, known_person_publisher, unknown_person_publisher, person_cache):
        """
        :type face_detection: FaceDetection.FaceDetection
        :type face_alignment: FaceAlignment.FaceAlignment
        :type face_embedder: FaceEmbedder.FaceEmbedder
        :type face_labeler: FaceLabeler.FaceLabeler
        :type face_filter: FaceFilter.FaceFilter
        :type unknown_face_labeler: UnknownFaceLabeler.UnknownFaceLabeler
        :type face_grouper: FaceGrouper.FaceGrouper
        :type session_memory: SessionMemory.SessionMemory
        :type known_person_publisher: rospy.Publisher
        :type unknown_person_publisher: rospy.Publisher
        """
        self.face_detection = face_detection
        self.face_alignment = face_alignment
        self.face_embedder = face_embedder
        self.face_labeler = face_labeler
        self.face_filter = face_filter
        self.unknown_face_labeler = unknown_face_labeler
        self.face_grouper = face_grouper
        self.session_memory = session_memory
        self.faces_detected_publisher = faces_detected_publisher
        self.known_person_publisher = known_person_publisher
        self.unknown_person_publisher = unknown_person_publisher
        self.person_cache = person_cache

    def _generate_header(self):
        h = Header()
        h.stamp = rospy.Time.now()  # Note you need to call rospy.init_node() before this will work
        return h

    def known_person_detected(self, person_id, record_timestamp):
        rospy.logerr("Entered: Known person id={0}".format(person_id))
        self.known_person_publisher.publish(self.person_cache[person_id])

    def unknown_person_detected(self, unknown_person_id, face_vectors, preview_image, record_timestamp):
        rospy.logerr("Entered: Unknown person id={0}".format(unknown_person_id))
        message = UnknownPersonEntered()
        message.person_id = unknown_person_id
        face_vector_messages = []
        for face_vector in face_vectors:
            face_vector_messages.append(FaceVector(face_vector))
        message.face_vectors = face_vector_messages
        message.preview_image = bridge.cv2_to_imgmsg(preview_image, encoding="passthrough")
        self.unknown_person_publisher.publish(message)

    def detect_person(self, face_image, person_image, record_timestamp):

        def handle_known_face(self, person_id, confidence, face_vector):
            self.face_grouper.update_known(person_id, confidence, face_vector, record_timestamp)
            if self.face_grouper.known_threshold_reached(person_id):
                self.face_grouper.known_reset(person_id)
                if not self.session_memory.known_contains(person_id):
                    self.known_person_detected(person_id, record_timestamp)
                self.session_memory.known_update(person_id, record_timestamp)

        def handle_unknown_face(self, face_vector, preview_image):
            unknown_person_id = self.unknown_face_labeler.label(face_vector)
            self.face_grouper.update_unknown(unknown_person_id, face_vector, record_timestamp)
            if self.face_grouper.unknown_threshold_reached(unknown_person_id):
                face_vectors = self.face_grouper.unknown_reset(unknown_person_id)
                if not self.session_memory.unknown_contains(unknown_person_id):
                    self.unknown_person_detected(unknown_person_id, face_vectors, preview_image, record_timestamp)
                self.session_memory.unknown_update(unknown_person_id, record_timestamp)

        def resize_bounding_box(bounding_box):
            """
            :param bounding_box: 
            :type bounding_box: dlib.rectangle
            :return: 
            """
            face_height, face_width, _ = face_image.shape
            person_height, person_width, _ = person_image.shape
            w_resize_factor = person_width / face_width
            h_resize_factor = person_height / face_height
            x, y, width, height = bounding_box
            return x * w_resize_factor, y * h_resize_factor, width * w_resize_factor, height * h_resize_factor

        boundboxes = self.face_detection.detect_faces(face_image)
        boundboxes = map(resize_bounding_box, boundboxes)

        faces = []
        face_bounding_boxes_msg = FaceBoundingBoxes()
        face_bounding_boxes_msg.header.stamp = rospy.Time.from_sec(record_timestamp)
        for x, y, width, height in boundboxes:
            faces.append(person_image[y:y+height,x:x+width])
            bounding_box = BoundingBox()
            bounding_box.x = x
            bounding_box.y = y
            bounding_box.width = width
            bounding_box.height = height
            face_bounding_boxes_msg.faces.append(bounding_box)

        self.faces_detected_publisher.publish(face_bounding_boxes_msg)

        aligned_faces = self.face_alignment.align_faces(person_image, boundboxes)
        embeddings = self.face_embedder.embed(aligned_faces)
        labeled_faces = self.face_labeler.label(embeddings)
        for face, (person_id, confidence, face_vector) in zip(faces, labeled_faces):
            rospy.loginfo("Detected: id={0} with confidence{1}".format(person_id, confidence))
            if self.face_filter.is_known(face_vector, confidence):
                handle_known_face(self, person_id, face_vector, confidence)
            elif self.face_filter.is_unknown(face_vector, confidence):
                handle_unknown_face(self, face_vector, face)
