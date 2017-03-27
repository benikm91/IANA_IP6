import openface

from FaceEmbedder import FaceEmbedder


class OpenFaceEmbedder(FaceEmbedder):

    def __init__(self, network_model, img_dim, cuda):
        """
        :param network_model: Path to the pre-trained FaceNet model
        :type network_model: str
        :param img_dim: Dimension of the incoming cropped face images
        :type img_dim: int
        :param cuda: Whether to run on gpu using cuda or not
        :type cuda: bool
        """
        self.net = openface.TorchNeuralNet(network_model, imgDim=img_dim, cuda=cuda)

    def embed(self, faces):
        """
        Embbeds all passed given faces in a 128 dimension hypersphere
        :param faces: Tuple of cropped faces and source face image
        :type faces: [(np.ndarray, np.ndarray)]
        :return: Tuple of embbedings and source face image
        :rtype: [(np.ndarray, np.ndarray)]
        """
        embeddings = []
        for cropped in faces:
            embeddings.append(self.net.forward(cropped))
        return embeddings
