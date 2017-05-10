import numpy as np

from person_detection.clusterer.FaceClusterer import FaceClusterer


class AverageCluster(object):
    def __init__(self, label, face_vector):
        self.label = label
        self.average = face_vector
        self.items = [face_vector]
        self.count = 1

    def add(self, item):
        self.average = (self.average * self.count + item) / (self.count + 1)
        self.items.append(item)
        self.count += 1


class AverageFaceClusterer(FaceClusterer):
    """
    :type distance: function
    :type threshold_same: float
    :type clusters: list[AverageCluster]
    :type next_label: int
    """
    def __init__(self, threshold_same, distance=lambda v0, v1: np.linalg.norm(v0 - v1), confidence=lambda dist: 1. - dist / 2.):
        self.threshold_same = threshold_same
        self.distance = distance
        self.confidence = confidence
        self.clusters = list()
        self.next_label = 0

    def __add_cluster(self, face_vector):
        """
        :param face_vector:
        :type face_vector: np.ndarray
        :return: The label of the new cluster
        :rtype: int
        """
        self.clusters.append(AverageCluster(self.next_label, face_vector))
        self.next_label += 1
        return self.next_label - 1

    def cluster(self, face_vector):
        """
        :param face_vector:
        :type face_vector: np.ndarray
        :return: The cluster label
        :rtype: int
        """
        if len(self.clusters) == 0:
            return self.__add_cluster(face_vector)
        else:
            closest_cluster = min(self.clusters, key=lambda b: self.distance(b.average, face_vector))
            if self.confidence(self.distance(closest_cluster.average, face_vector)) >= self.threshold_same:
                closest_cluster.add(face_vector)
                return closest_cluster.label
            else:
                return self.__add_cluster(face_vector)

    def get_biggest(self):
        """
        :return:
        :rtype: AverageCluster
        """
        if self.count() == 0:
            return None
        return max(self.clusters, key=lambda v: v.count)

    def count(self):
        return len(self.clusters)
