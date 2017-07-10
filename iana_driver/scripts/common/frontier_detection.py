import rospy
import numpy as np
import geometry_msgs.msg
import nav_msgs.msg
from collections import deque


def pose_to_map_point(pose, resolution):
    return pose.position.x / resolution, pose.position.y / resolution


def pose_to_relative_map_point(pose, origin, resolution):
    return origin.position.x + pose.position.x / resolution, origin.position.y + pose.position.y / resolution


def map_point_to_relative_pose(point, origin, resolution):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = origin.position.x + point[0] * resolution
    pose.position.y = origin.position.y + point[1] * resolution
    return pose


def map_point_to_relative_pose_with_euclidean_distance(point, origin, resolution):
    pose = map_point_to_relative_pose(point, origin, resolution)
    a = pose.position.x - origin.position.x
    b = pose.position.y - origin.position.y
    distance = np.sqrt(a*a+b*b)
    return pose, distance


def find_frontier_points_in_map(start, grid_map):
    processed = -2
    unknown = -1
    occupied_threshold = 50
    neighbours = [
        [1, 1],
        [1, -1],
        [-1, -1],
        [-1, 1]
    ]
    frontiers = []
    rospy.loginfo("start: {} = {}".format(start, grid_map[start]))
    if grid_map[start] == unknown or grid_map[start] >= occupied_threshold:
        return frontiers
    queue = deque()
    queue.append(start)
    grid_map[start] = processed
    while len(queue) > 0:
        curr = queue.popleft()
        for neighbour in neighbours:
            coord = (curr[0] + neighbour[0], curr[1] + neighbour[1])
            if 0 <= grid_map[coord] < occupied_threshold:
                grid_map[coord] = processed
                queue.append(coord)
            elif grid_map[coord] == unknown:
                frontiers.append(coord)


def find_frontier_points_in_occupancy_grid(occupancy_grid):
    grid_map = np.reshape(occupancy_grid.data, (occupancy_grid.info.height, occupancy_grid.info.width))
    origin = occupancy_grid.info.origin
    resolution = occupancy_grid.info.resolution
    start = pose_to_map_point(origin, resolution)
    frontiers = find_frontier_points_in_map(start, grid_map)
    return [map_point_to_relative_pose(x, origin, resolution) for x in frontiers]


def find_closest_frontier_point_in_occupancy_grid(occupancy_grid, min_distance):
    """
    :param occupancy_grid:
    :type occupancy_grid: nav_msgs.msg.OccupancyGrid
    :param min_distance: The min distance of the frontier point in m
    :type min_distance: float
    :return:
    :rtype: geometry_msgs.msg.Pose
    """
    rospy.loginfo("find_closest_frontier_point_in_occupancy_grid started")
    grid_map = np.reshape(occupancy_grid.data, (occupancy_grid.info.height, occupancy_grid.info.width))
    rospy.loginfo("grid_map = {}".format(grid_map))
    origin = occupancy_grid.info.origin
    resolution = occupancy_grid.info.resolution
    start = pose_to_map_point(origin, resolution)
    frontiers = find_frontier_points_in_map(start, grid_map)
    rospy.loginfo("frontiers = {}".format(frontiers))
    frontiers_with_distance = [map_point_to_relative_pose_with_euclidean_distance(x, origin, resolution) for x in frontiers]
    frontiers_with_distance.sort(cmp=lambda a, b: a[1] - b[1])
    for pose_with_distance in frontiers_with_distance:
        if pose_with_distance[1] >= min_distance:
            rospy.loginfo("pose = {}".format(pose_with_distance[0]))
            return pose_with_distance[0]
    rospy.loginfo("No pose!!!")
    return None
