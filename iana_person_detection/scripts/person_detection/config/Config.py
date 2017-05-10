class Config:
    """
    Makes anonymous objects possible in Python 2.7
    """

    class CameraMode:
        NOTEBOOK_CAMERA = 0
        WEBCAM_CAMERA = 1

    class LoggingStrategy:
        NO_LOGGING = 0
        CONSOLE = 1
        FILE = 2

    def __init__(self, **entries): self.__dict__.update(entries)

    def __eq__(self, r2): return self.__dict__ == r2.__dict__

    def __ne__(self, r2): return self.__dict__ != r2.__dict__
