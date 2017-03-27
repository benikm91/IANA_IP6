import scipy.spatial
from collections import defaultdict

from classifier.DistanceToAverageFaceClassifier import DistanceToAverageFaceClassifier


class CosineSimilarityClassifier(DistanceToAverageFaceClassifier):
    """
    Face embedding classifier using cosine distance as measure of distance
    :type faces: dict[int, Face]
    """

    def __init__(self):
        super(CosineSimilarityClassifier, self).__init__(scipy.spatial.distance.cosine, lambda min_dist: 1. - (min_dist / 2.))
        self.faces = defaultdict(lambda: None)
