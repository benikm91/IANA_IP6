from sklearn.grid_search import GridSearchCV
from sklearn.svm import SVC

from classifier.FaceClassifier import FaceClassifier


class SVMClassifier(FaceClassifier):

    def __init__(self):
        self.svm = None
        self.y = []
        self.X = []
        self.num_persons = 0

    def __train_svm(self):
        num_identities = len(set(self.y + [-1]))
        self.num_persons = num_identities - 1

        if num_identities <= 2:
            return

        param_grid = [
            {'C': [1, 10, 100, 1000],
             'kernel': ['linear']},
            {'C': [1, 10, 100, 1000],
             'gamma': [0.001, 0.0001],
             'kernel': ['rbf']}
        ]
        self.svm = GridSearchCV(SVC(C=1), param_grid, cv=5).fit(self.X, self.y)
        print "okidoki"

    def train(self, face_data):
        for person_id, feature_vectors in face_data:
            for feature_vector in feature_vectors:
                self.y.append(person_id)
                self.X.append(feature_vector)
                print "{} : {}".format(person_id, feature_vector)
        self.__train_svm()

    def update(self, person_id, feature_vectors):
        for feature_vector in feature_vectors:
            self.y.append(person_id)
            self.X.append(feature_vector)
        self.__train_svm()

    def predict(self, face_embedding):
        if self.num_persons == 0:
            return -1, -float("inf")
        elif self.num_persons == 1:
            return self.y[0], 1
        else:
            return self.svm.predict(face_embedding)[0], 1

