import heapq
import math
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


def is_in_contact_with_obstacle(map, robot: list[float], sensor_range: int = 2) -> bool:
    r_x, r_y = robot
    map_max_y, map_max_y_row = next(iter(map.items()))
    map_max_x = len(map_max_y_row)
    for i in range(-sensor_range, sensor_range):
        for j in range(-sensor_range, sensor_range):
            if i == 0 and j == 0:
                continue
            x = int(r_x) + i
            y = int(r_y) + j
            if (0 <= x < map_max_x and 0 <= y < map_max_y and map[x][y] == 'o'):
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


def lazy_dijkstra(graph, root):
    # https://pythonalgos.com/dijkstras-algorithm-in-5-steps-with-python/
    n = len(graph)
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
    for dest, path in enumerate(paths):
        current = dest
        while current != root:
            path.append(current)
            current = previous[current]
        path.append(root)

    return dist, paths
