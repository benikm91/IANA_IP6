from collections import defaultdict

import cv2
import dlib
import networkx as nx
import time

from face_detection_validation.face_detection.DlibFaceDetection import DlibFaceDetection

predictor_path='/home/benjamin/IP6/src/iana_person_detection/scripts/res/models/dlib/shape_predictor_68_face_landmarks.dat'
img_dim=96

# no debugging
debug_condition = False
# show all images
#debug_condition = True
# show all images with false positives
# debug_condition = lambda image_name, tp, fp, fn: fp > 0
# show only one specific image
#debug_condition = lambda image_name, tp, fp, fn: image_name == '2002/09/01/big/img_16225'

folder = "/home/benjamin/Desktop/face_detection"
files = [
    "/FDDB-folds/FDDB-fold-01-ellipseList.txt",
    "/FDDB-folds/FDDB-fold-02-ellipseList.txt",
    "/FDDB-folds/FDDB-fold-03-ellipseList.txt",
    "/FDDB-folds/FDDB-fold-04-ellipseList.txt",
    "/FDDB-folds/FDDB-fold-05-ellipseList.txt",
    "/FDDB-folds/FDDB-fold-06-ellipseList.txt",
    "/FDDB-folds/FDDB-fold-07-ellipseList.txt",
    "/FDDB-folds/FDDB-fold-08-ellipseList.txt",
    "/FDDB-folds/FDDB-fold-09-ellipseList.txt",
    "/FDDB-folds/FDDB-fold-10-ellipseList.txt",
]

def main():
    haar = HaarFaceDetector(cv2.CascadeClassifier("/home/benjamin/IP6/src/iana_person_detection/scripts/res/models/opencv/lbpcascade_frontalface.xml.xml"))
    #dlib = DlibFaceDetector(DlibFaceDetection(predictor_path))


    print "number_of_images, number_of_faces, tp, fp, fn"
    start = time.time()
    total = (0, 0, 0, 0, 0)
    runs = 10
    for i in range(0, runs):
        for file in files:
            validation = Validation("",
                                    folder,
                                    #"/Users/benikm91/Documents/FHNW/Semester5/projekt/datasets/FDDB/FDDB-folds/FDDB-fold-sample.txt")
                                    folder+file)
            result = validation.run(haar)
            print result
            total = tuple(map(sum, zip(total, result)))
    print (time.time() - start) / runs
    print "Total"
    print total

def debug_show_image(title, path, bounding_boxes):
    image = cv2.imread(path)
    for bb in bounding_boxes:
        cv2.rectangle(image, (bb.left(), bb.top()), (bb.right(), bb.bottom()), (0,255,0), 3)
    cv2.imshow(title, image)


def bounding_box_to_dlib_rectangle(bounding_box):
    x, y, width, height = bounding_box
    return dlib.rectangle(long(x), long(y), long(x + width), long(y + height))


class FaceDetector(object):

    def face_detection(self, img):
        pass


class DlibFaceDetector(FaceDetector):

    def __init__(self, face_detector):
        self.face_detector = face_detector

    def face_detection(self, img):
        return self.face_detector.detect_faces(img)


class HaarFaceDetector(FaceDetector):

    def __init__(self, face_cascade):
        self.face_cascade = face_cascade

    def face_detection(self, img):
        return map(bounding_box_to_dlib_rectangle, self.face_cascade.detectMultiScale(img, 1.3, 5))


class Validation:

    def __init__(self, name, images_path, solution_path):
        self.name = name
        self.images_path = images_path
        with open(solution_path) as f:
            content = f.readlines()
        content = [x.strip() for x in content]
        contentIndex = 0
        self.solutions = defaultdict(lambda: list())
        while contentIndex < len(content):
            # read name
            name = content[contentIndex]
            contentIndex += 1
            # read number of faces
            number_of_faces = int(content[contentIndex])
            contentIndex += 1
            # read faces and add them to solutions
            for x in range(0, number_of_faces):
                ellipse = content[contentIndex]
                self.solutions[name].append(self.to_rect(ellipse))
                contentIndex += 1

    @staticmethod
    def to_rect(ellipse):
        ra, rb, theta, cx, cy, s = map(float, ellipse.split())
        left = int(cx - rb)
        right = int(cx + rb)
        top = int(cy - ra)
        bottom = int(cy + ra)
        return dlib.rectangle(left=left, top=top, right=right, bottom=bottom)

    def run(self, face_detection):
        detected_faces = defaultdict(lambda: list())
        for image_name in self.solutions.keys():
            file_path = "{:s}/{:s}.jpg".format(self.images_path, image_name)
            image = cv2.imread(file_path)
            bounding_boxes = face_detection.face_detection(image)
            detected_faces[image_name] = bounding_boxes
        return self.total_score(detected_faces, self.solutions)

    @staticmethod
    def total_score(detected, solution):
        def score(detected_bb, solution_bb):
            threshold = 0.5
            G = nx.Graph()
            G.add_nodes_from(detected_bb)
            G.add_nodes_from(solution_bb)
            G.add_node("trash")
            for detected in detected_bb:
                for solution in solution_bb:
                    overlapping = float(detected.intersect(solution).area()) / detected.area()
                    G.add_edge(detected, solution, weight=overlapping)
                # If no better edge was added, add a edge to trash with just to little of a value => fn
                G.add_edge(detected, "trash", weight=threshold)
            assert nx.is_bipartite(G)
            tp = 0
            fp = 0
            for a, b in nx.max_weight_matching(G).iteritems():
                if threshold < G[a][b]['weight']:
                    tp += 1
                else:
                    fp += 1
            tp /= 2 # Matching checks both (!) directions
            fp /= 2 # Matching checks both (!) directions
            fn = len(solution_bb) - tp # not detected faces are false negatives
            return tp, fp, fn
        tp, fp, fn = 0, 0, 0
        number_of_faces = 0
        for key in solution.keys():
            detected_bb = detected[key]
            solution_bb = solution[key]
            tp_delta, fp_delta, fn_delta = score(detected_bb, solution_bb)
            # debug stuff:
            if (not callable(debug_condition) and debug_condition) or (callable(debug_condition) and debug_condition(key, tp_delta, fp_delta, fn_delta)):
                file_path = "{:s}/{:s}.jpg".format(folder, key)
                debug_show_image('detected', file_path, detected_bb)
                debug_show_image('solution', file_path, solution_bb)
                cv2.waitKey(0)
            tp, fp, fn = tp + tp_delta, fp + fp_delta, fn + fn_delta
            number_of_faces += len(solution_bb)
        return len(solution.keys()), number_of_faces, tp, fp, fn

if __name__ == "__main__":
    main()
