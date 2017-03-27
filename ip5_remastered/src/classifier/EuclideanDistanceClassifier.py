import numpy
from collections import defaultdict

from classifier.DistanceToAverageFaceClassifier import DistanceToAverageFaceClassifier


class EuclideanDistanceClassifier(DistanceToAverageFaceClassifier):
    """
    Face embedding classifier using cosine distance as measure of distance
    :type faces: dict[int, Face]
    """

    def __init__(self):
        super(EuclideanDistanceClassifier, self).__init__(lambda a, b: numpy.linalg.norm(a - b), lambda min_dist: 1. - min_dist / 2.)
        self.faces = defaultdict(lambda: None)
