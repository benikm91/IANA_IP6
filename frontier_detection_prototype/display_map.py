import cv2
import numpy as np
from collections import deque

map_file_name = "map_10.pgm"

color_unknown = (205, 205, 205)
color_free = (254, 254, 254)
color_occupied = (0, 0, 0)

robot_position = (240, 300)

map_orig = cv2.imread(map_file_name)
map_work = map_orig.copy()
map_frontier = map_orig.copy()

neighbours = [
    [1, 1],
    [1, -1],
    [-1, -1],
    [-1, 1]
]

def findFrontiers(start):
    if start == color_unknown or start == color_occupied:
        return []
    frontiers = []
    queue = deque()
    queue.append(start)
    map_work[start] = (100, 100, 100)
    while len(queue) > 0:
        curr = queue.popleft()
        for neighbour in neighbours:
            coord = (curr[0] + neighbour[0], curr[1] + neighbour[1])
            coordColor = tuple(map_work[coord])
            if coordColor == color_free:
                map_work[coord] = (100, 100, 100)
                queue.append(coord)
            elif coordColor == color_unknown:
                frontiers.append(coord)
    return frontiers

fronts = findFrontiers(robot_position)

for frontPoint in fronts:
    cv2.circle(map_frontier, (frontPoint[1], frontPoint[0]), 2, (0, 0, 255), -1)

def euclidean_distance_robot_position(point):
    a = robot_position[0] - point[0]
    b = robot_position[1] - point[1]
    return np.sqrt(a*a+b*b)

frontierPointDistances = map(euclidean_distance_robot_position, fronts)
min_index = np.argmin(frontierPointDistances)
minFrontierPoint = fronts[min_index]

cv2.circle(map_frontier, (minFrontierPoint[1], minFrontierPoint[0]), 5, (255, 0, 0), -1)

cv2.circle(map_frontier, (robot_position[1], robot_position[0]), 5, (0, 255, 0), -1)

cv2.imshow("original map", map_orig)
cv2.waitKey(0)

cv2.imshow("frontier map", map_frontier)
cv2.waitKey(0)

