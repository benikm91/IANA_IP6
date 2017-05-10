class FaceFilter(object):
    """
    Detect faces in an image and add possible event
    """

    def __init__(self, threshold_known, threshold_unknown):
        self.threshold_known = threshold_known
        self.threshold_unknown = threshold_unknown

    def is_known(self, face, confidence):
        return confidence >= self.threshold_known

    def is_unknown(self, face, confidence):
        return confidence <= self.threshold_unknown
