import heapq
import math
import numpy as np
from collections import defaultdict
from math import pi


def distance(pos1, pos2):
    (x1, y1) = pos1
    (x2, y2) = pos2
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def get_angle(i, j, k):
    i_x, i_y = i
    j_x, j_y = j
    k_x, k_y = k

    theta_i_j = math.atan2(j_y - i_y, j_x - i_x)
    theta_i_k = math.atan2(k_y - i_y, k_x - i_x)
    theta_i_j_k = theta_i_k - theta_i_j

    if theta_i_j_k < -pi:
        return theta_i_j_k + 2 * pi
    elif -pi <= theta_i_j_k < pi:
        return theta_i_j_k
    else:
        return theta_i_j_k - 2 * pi


def is_in_contact_with_obstacle(map_, robot: list[float], sensor_range: int = 2) -> bool:
    r_x, r_y = robot
    map_max_y, map_max_y_row = next(iter(map_.items()))
    map_max_x = len(map_max_y_row)
    for i in range(-sensor_range, sensor_range):
        for j in range(-sensor_range, sensor_range):
            if i == 0 and j == 0:
                continue
            x = int(r_x) + i
            y = int(r_y) + j
            if 0 <= x < map_max_x and 0 <= y < map_max_y and map_[x][y] == 'o':
                return True
    return False


def no_visible_robots_in_activated_touch_sensor_angle(robots: list[list], robot: list) -> bool:
    # Check if there are no robots in the touch sensor angle
    return True


def get_graph(one_simplices, fence_subcomplex):
    graph = defaultdict(list)
    for one_simplex in one_simplices:
        if one_simplex in fence_subcomplex['obstacle_simplices']:
            graph[one_simplex[1]].append([one_simplex[0], 2])
            graph[one_simplex[0]].append([one_simplex[1], 2])
        else:
            graph[one_simplex[0]].append([one_simplex[1], 1])
            graph[one_simplex[1]].append([one_simplex[0], 1])

    return graph


def lazy_dijkstra(graph, root, n):
    # https://pythonalgos.com/dijkstras-algorithm-in-5-steps-with-python/
    # set up "inf" distances
    dist = [math.inf for _ in range(n)]
    previous = {}
    # set up root distance
    dist[root] = 0
    # set up visited node list
    visited = [False for _ in range(n)]
    # set up priority queue
    pq = [(0, root)]
    # while there are nodes to process
    while len(pq) > 0:
        # get the root, discard current distance
        _, u = heapq.heappop(pq)
        # if the node is visited, skip
        if visited[u]:
            continue
        # set the node to visited
        visited[u] = True
        # check the distance and node and distance
        for v, l in graph[u]:
            # if the current node's distance + distance to the node we're visiting
            # is less than the distance of the node we're visiting on file
            # replace that distance and push the node we're visiting into the priority queue
            if dist[u] + l < dist[v]:
                dist[v] = dist[u] + l
                previous[v] = u
                heapq.heappush(pq, (dist[v], v))

    paths = [[] for _ in range(n)]
    if previous == {}:
        paths[root].append(root)
    else:
        for dest, path in enumerate(paths):
            current = dest
            while current != root:
                path.append(current)
                current = previous[current]
            path.append(root)

    return dist, paths


# ------------------------- get_fence_subcomplex functions  ---------------------------#

def is_obstacle_simplex(robots, map_, one_simplex) -> bool:
    """
    Check if robots {i,j} are an obstacle simplex at a convex corner:
    We get the closest fence 1 simplex attached to i or j, say its {i,k} and calculate the angle theta ijk between them.
    If it is less than pi/3 than the robots j and k do not see each other due to occlusion by an obstacle
    """
    i, j = one_simplex
    if (is_in_contact_with_obstacle(map_, robots[i]) and
            is_in_contact_with_obstacle(map_, robots[j]) and
            no_visible_robots_in_activated_touch_sensor_angle(robots, robots[i]) and
            no_visible_robots_in_activated_touch_sensor_angle(robots, robots[j])):
        return True
    return False


def get_deployment_absolute_position(robot_a_coordinate: list, robot_b_coordinate: list,
                                     deployment_angle: float) -> list:
    x1, y1 = robot_a_coordinate
    x2, y2 = robot_b_coordinate
    x3 = math.cos(deployment_angle) * (x2 - x1) - math.sin(deployment_angle) * (y2 - y1) + x1
    y3 = math.sin(deployment_angle) * (x2 - x1) + math.cos(deployment_angle) * (y2 - y1) + y1
    return [x3, y3]


def get_deployment_angle(obstacle_simplices, robots: list, one_simplex: list,
                         one_simplices: list, uncov: list) -> float:
    theta_i_j_new, theta_j_i_new = [], []
    i, j = one_simplex
    for sigma in uncov:
        S_i, S_j = get_closest_fence_candidates(robots, one_simplex, one_simplices, sigma)
        if not S_i:
            theta_i_j_new.append(sigma * pi / 3)
        else:
            k_i = min([[k, abs(get_angle(robots[i], robots[j], robots[k]))] for k in S_i], key=lambda x: x[1])[0]
            theta_i_j_k_i = get_angle(robots[i], robots[j], robots[k_i])
            if abs(theta_i_j_k_i) < pi / 3:
                obstacle_simplices.append([i, k_i])  # TODO: Check this!
            else:
                theta_i_j_new.append(sigma * min([pi / 3, abs(theta_i_j_k_i / 2)]))

        if not S_j:
            theta_j_i_new.append(-sigma * pi / 3)
        else:
            k_j = min([[k, abs(get_angle(robots[j], robots[i], robots[k]))] for k in S_j], key=lambda x: x[1])[0]
            theta_j_i_k_j = get_angle(robots[j], robots[i], robots[k_j])

            if abs(theta_j_i_k_j) < pi / 3:
                obstacle_simplices.append([j, k_j])  # TODO: Check this!
            else:
                theta_j_i_new.append(-sigma * min([pi / 3, abs(theta_j_i_k_j / 2)]))

    return theta_i_j_new, theta_j_i_new


def get_closest_fence_candidates(robots: list, one_simplex: list, one_simplices: list[list], sigma: int):
    S_i, S_j = [], []
    i, j = one_simplex
    for l1, l2 in one_simplices:
        if [l1, l2] == [i, j]:
            continue

        if i in [l1, l2]:
            l_i = l1 if i == l2 else l2
            if np.sign(get_angle(robots[i], robots[j], robots[l_i])) == sigma:
                S_i.append(l_i)

        if j in [l1, l2]:
            l_j = l2 if l1 == j else l1
            if np.sign(get_angle(robots[j], robots[i], robots[l_j])) == -sigma:
                S_j.append(l_j)

    return S_i, S_j


def get_one_simplex_uncov(robots, one_simplex: list, two_simplices: list[list]) -> list:
    previous_angle, current_angle = None, None
    i, j = one_simplex
    for two_simplex in two_simplices:
        diff = [x for x in two_simplex if x not in one_simplex]
        if len(diff) == 1:  # Checks 2 simplices that {i, j, k_u} that have one simplex {i, j} as their vertices
            k_u = diff[0]
            current_angle = get_angle(robots[i], robots[j], robots[k_u])
            if previous_angle is None:
                previous_angle = current_angle
                continue
            if np.sign(previous_angle) != np.sign(current_angle):
                return []

    if current_angle is None:
        return [-1, 1]

    if np.sign(current_angle) == 0:
        return [-1, 1]
    elif np.sign(current_angle) > 0:
        return [-1]
    else:
        return [1]


def filter_one_simplices_exception(one_simplices: list[list]) -> dict:
    # TODO: To be implemented later
    return one_simplices, []
