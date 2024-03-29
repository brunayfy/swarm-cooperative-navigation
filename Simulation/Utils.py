import heapq
import math
from collections import defaultdict
from copy import deepcopy
from dataclasses import dataclass
from math import pi

import numpy as np


@dataclass
class Simplex:
    zero_simplices: list[int]
    one_simplices: list[int]
    two_simplices: list[int]


@dataclass
class FenceSubcomplex:
    obstacle_simplices: list[list[int]]
    frontier_simplices: list[list[int]]


@dataclass
class Map:
    boundary: list[list[float]]
    obstacles: list[list[list[float]]]


def get_pair_combinations(list_: list) -> list:
    combinations = []
    for i, e_1 in enumerate(list_):
        for e_2 in list_[i + 1 :]:
            combinations.append([e_1, e_2])
    
    return combinations

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


def get_graph(one_simplices: list[list[int]], fence_subcomplex: FenceSubcomplex, robot_is_obstacle) -> \
        defaultdict[list[list[int]]]:
    graph = defaultdict(list[list[int]])
    obstacle_weight = 4
    for one_simplex in one_simplices:
        if one_simplex in fence_subcomplex.obstacle_simplices:
            graph[one_simplex[1]].append([one_simplex[0], obstacle_weight])
            graph[one_simplex[0]].append([one_simplex[1], obstacle_weight])
        else:
            # Note: Adding weight 'obstacle_weight' for robots in contact with obstacle as well
            graph[one_simplex[1]].append([one_simplex[0], obstacle_weight if robot_is_obstacle[one_simplex[0]] else 1])
            graph[one_simplex[0]].append([one_simplex[1], obstacle_weight if robot_is_obstacle[one_simplex[1]] else 1])

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
    pq = [(0.0, root)]
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

def ensure_valid_deploy_position(
        sim_map: Map, 
        robots: list[list],
        current_position: list[float], 
        deploy_positions: list[list], 
        sigma: float,
        obstacle_radius: float = 0.3, 
        margin: float = 0.05
    ):
    """
    a. Check if the deployment position is:
        1. valid(it's not inside obstacle or outside map): keep position,
            else move to the closest valid position(-margin) - make
        2. is near(obstacle_radius) walls or obstacle: is_obstacle == True
    """

    if deploy_positions == []:
        raise ValueError("The robot chosen by dijkstra doesn't have a deployment angle. Implement filtration on "
        "skeleton construction so that it doesn't choose this paths")

    for deploy_position in deploy_positions:
        cx, cy = current_position
        dx, dy = deploy_position
        (m_x1, m_y1), (m_x2, m_y2) = sim_map.boundary
        is_obstacle = False

        # check if new deploy is inside map
        a_den = dx - cx
        a = 0 if a_den == 0 else (dy - cy) / a_den
        b = cy - a * cx
        if dx <= m_x1:
            dx = m_x1 + margin
            is_obstacle = True
        elif dx >= m_x2:
            dx = m_x2 - margin
            is_obstacle = True
        elif dx <= m_x1 + obstacle_radius or dx >= m_x2 - obstacle_radius:
            is_obstacle = True  # robot near map wall

        if dy <= m_y1:
            dy = m_y1 + margin
            is_obstacle = True
        elif dy >= m_y2:
            dy = m_y2 - margin
            is_obstacle = True
        elif dy <= m_y1 + obstacle_radius or dy >= m_y2 - obstacle_radius:
            is_obstacle = True  # robot near map wall

        # check if new deploy is inside obstacles or if robot is near obstacle
        a_den = dx - cx
        a = 0 if a_den == 0 else (dy - cy) / a_den
        b = cy - a * cx
        for (o_x1, o_y1), (o_x2, o_y2) in sim_map.obstacles:
            if o_x1 <= dx <= o_x2:
                if o_y1 <= dy <= o_y2:
                    if cx <= o_x1:
                        dx = o_x1 - margin
                        is_obstacle = True
                    elif cx >= o_x2:
                        dx = o_x2 + margin
                        is_obstacle = True
                    if cy <= o_y1:
                        dy = o_y1 - margin
                        is_obstacle = True
                    elif cy >= o_y2:
                        dy = o_y2 + margin
                        is_obstacle = True
                elif o_y1 - obstacle_radius <= dy <= o_y2 + obstacle_radius:
                    is_obstacle = True
            elif o_x1 - obstacle_radius <= dx <= o_x2 + obstacle_radius and \
                    o_y1 - obstacle_radius <= dy <= o_y2 + obstacle_radius:
                is_obstacle = True  # robot near obstacle

        # check if deploy is after a obstacle
        for (o_x1, o_y1), (o_x2, o_y2) in sim_map.obstacles:
            if (
                ( 
                    (cx < o_x1 and dx > o_x2) or
                    (dx < o_x1 and cx > o_x2) 
                ) and
                o_y1 <= cy <= o_y2 and 
                o_y1 <= dy <= o_y2
            ):
                is_obstacle = True
                if dx <= o_x1:
                    dx = o_x2 + margin
                elif dx >= o_x2:
                    dx = o_x1 - margin
            elif (
                ( 
                    (cy < o_y1 and dy > o_y2) or
                    (dy < o_y1 and cy > o_y2) 
                ) and
                o_x1 <= cx <= o_x2 and 
                o_x1 <= dx <= o_x2
            ):
                is_obstacle = True
                if dy <= o_y1:
                    dy = o_y2 + margin
                elif dy >= o_y2:
                    dy = o_y1 - margin
        
        # check if new deploy position is obstructed
        [dx, dy], is_obstacle_from_obstruction = check_obstruction_between_robots([cx, cy], [dx, dy], sim_map.obstacles, margin)
        is_obstacle = is_obstacle or is_obstacle_from_obstruction
        
        continue_outer_loop = False
        for r in robots:
            if distance(r, [dx, dy]) <= sigma:
                continue_outer_loop = True
                break
        if continue_outer_loop:
            continue

        return [dx, dy], is_obstacle
        
    else:
        raise ValueError("The valid deploy position is too close to the current position.")


def is_obstacle_simplex(one_simplex: list[int], robot_is_obstacle: defaultdict[bool]) -> bool:
    """
    Check if robots {i,j} are an obstacle simplex:
        If robots i and j are in touch with obstacles(consider that there are no robots visible in the direction of
        the activated touch sensor because the robots cannot collide in this simulation) and the contact angle is in
        the expanding direction uncov, them 1-simplex {i,j} is an obstacle simplex.
    """
    i, j = one_simplex
    # TODO: Calculate the contact angle with the obstacle to know if its on the same side of uncov
    if robot_is_obstacle[i] and robot_is_obstacle[j]:
        return True
    return False


def get_deployment_absolute_position(robot_a_coordinate: list, robot_b_coordinate: list,
                                     deployment_angle: float, r: float) -> list:
    
    a, b = robot_a_coordinate
    c, d = robot_b_coordinate
    rot = np.array(( (np.cos(deployment_angle), -np.sin(deployment_angle)),
                     (np.sin(deployment_angle),  np.cos(deployment_angle)) ))
    
    ab_vec = np.array([c-a, d-b])
    ab_vec_norm = (ab_vec / np.linalg.norm(ab_vec))
    rotated_scaled_vector = rot.dot(ab_vec_norm) * r
    x = rotated_scaled_vector[0] + a
    y = rotated_scaled_vector[1] + b
    return [x, y]


def get_deployment_angle(obstacle_simplices, robots: list, one_simplex: list,
                         one_simplices: list, uncov: list, beta: float) -> tuple[list[float], list[float]]:
    theta_i_j_new, theta_j_i_new = [], []
    i, j = one_simplex
    for sigma in uncov:
        S_i, S_j = get_closest_fence_candidates(robots, one_simplex, one_simplices, sigma)
        if not S_i:
            theta_i_j_new.append(sigma * pi / 3)
        else:
            k_i = min([[k, abs(get_angle(robots[i], robots[j], robots[k]))] for k in S_i], key=lambda x: x[1])[0]
            theta_i_j_k_i = get_angle(robots[i], robots[j], robots[k_i])
            if abs(theta_i_j_k_i) < pi / 3 - 2 * beta:
                if sorted([j,k_i]) not in one_simplices:
                    obstacle_simplices.append(sorted([i, k_i]))
                    obstacle_simplices.append([i,j])
            else:
                theta_i_j_new.append(sigma * min([pi / 3, abs(theta_i_j_k_i / 2)]))

        if not S_j:
            theta_j_i_new.append(-sigma * pi / 3)
        else:
            k_j = min([[k, abs(get_angle(robots[j], robots[i], robots[k]))] for k in S_j], key=lambda x: x[1])[0]
            theta_j_i_k_j = get_angle(robots[j], robots[i], robots[k_j])

            if abs(theta_j_i_k_j) < pi / 3 - 2 * beta :
                if sorted([i,k_j]) not in one_simplices:
                    obstacle_simplices.append(sorted([j, k_j])) 
                    obstacle_simplices.append([i,j])

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


def point_inside_line(px, py, ax, ay, bx, by):
    return (ax <= px <= bx or bx <= px <= ax) and (by <= py <= ay or ay <= py <= by)


def find_robot_neighbors(robot:int, one_simplices: list[list[int]]):
    k_neighbors = []
    for _one_simplex in one_simplices:
        diff = [x for x in _one_simplex if x != robot]
        if len(diff) == 1:
            k_neighbors.append(diff[0])
    return k_neighbors

def remove_one_simplex(one_simplex, two_simplices):
    two_simplices_without_exception = []
    for two_simplex in two_simplices:
        diff = [x for x in one_simplex if x in two_simplex]
        if len(diff) != 2:
            two_simplices_without_exception.append(two_simplex)
    return two_simplices_without_exception

def filter_exceptions(robots: list, simplices: dict) -> tuple[list, list]:
    # We have to remove the false positive fences (when uncov !=0, but it is not fence)
    # This filter could also be applied after finding out the fence subcomplex
    normal = {1: deepcopy(simplices[1]), 2: deepcopy(simplices[2])}
    exception = []
    for one_simplex in deepcopy(simplices[1]):
        if one_simplex in exception:
            continue

        for two_simplex in normal[2]:
            diff = [x for x in two_simplex if x not in one_simplex]
            if len(diff) == 1:
                break
        else:
            continue

        i, j = one_simplex
        # Taking first neighbor to help find if there is an exception. Maybe should analyze more neighbors if no exception is found
        k = diff[0]
        k_neighbors = [x for x in find_robot_neighbors(k, normal[1]) if x!= i and x!=j]

        theta_k_ij = get_angle(robots[k], robots[i], robots[j])
        theta_k_ij_sign = np.sign(theta_k_ij)
        possible_exceptions = []
        for neighbor in k_neighbors:
            # If there is a crossing happening between {i,j} and {k,k_neighbor} than the sum of angles will be of the same sign
            # which means they are not supposed to be in the fence subcomplex.
            if (
                    theta_k_ij_sign == np.sign(get_angle(robots[k], robots[i], robots[neighbor])) and
                    theta_k_ij_sign == np.sign(get_angle(robots[k], robots[neighbor], robots[j])) and
                    # This check is to see if the two simplex being analized is inside smaller two simplicex. If it is the neighbor and k will be on the same side of the edge, meaning that they dont cross the edge. 
                    np.sign(get_angle(robots[i], robots[j], robots[k])) != np.sign(get_angle(robots[i], robots[j], robots[neighbor]))
            ):
                # TODO: Optional improvement, compare the one_simplex to the neighbor simplex it is overlapping. Add to the exeption the biggest edge(the one_simplex were the robots are farthest away)
                possible_exceptions.append(sorted([k, neighbor]))
        
        if possible_exceptions != []:
            possible_exceptions.append(one_simplex)
            higher_distance_exception = max(possible_exceptions, key=lambda x: distance(robots[x[0]], robots[x[1]]))
            if higher_distance_exception in normal[1]:
                normal[1].remove(higher_distance_exception)
            exception.append(higher_distance_exception)
            normal[2] = remove_one_simplex(higher_distance_exception, normal[2])

    return exception, normal[1], normal[2]


def check_obstruction_between_robots(current_robot, deploy_robot, obstacles, margin):
    # Check if one-simplices cross an obstacle(robots cannot see each other)
    c_x, c_y = current_robot
    d_x, d_y = deploy_robot

    # Finding line equation : y = ax + b
    a_den = d_x - c_x
    if a_den != 0:
        a = (d_y - c_y) / a_den
        b = c_y - a * c_x

    for (obs_x1, obs_y1), (obs_x2, obs_y2) in obstacles:
        if a_den == 0:
            # two robots are vertically aligned
            if obs_x1 <= c_x <= obs_x2 and point_inside_line(c_x, obs_y1, c_x, c_y, d_x, d_y):
                return True
        else:
            y1 = a * obs_x1 + b
            if obs_y1 <= y1 <= obs_y2 and point_inside_line(obs_x1, y1, c_x, c_y, d_x, d_y):
                if d_y > obs_y2 or c_y > obs_y2:
                    if d_y > c_y:
                        d_x = obs_x1 - margin
                    else:
                        d_y = obs_y2 + margin
                else:
                    if d_y > c_y:
                        d_y = obs_y1 - margin
                    else:
                        d_x = obs_x1 - margin

                return [d_x, d_y], True
            else:
                y2 = a * obs_x2 + b
                if obs_y1 <= y2 <= obs_y2 and point_inside_line(obs_x2, y2, c_x, c_y, d_x, d_y):
                    if d_y > obs_y2 or c_y > obs_y2:
                        if d_y > c_y:
                            d_x = obs_x2 + margin
                        else:
                            d_y = obs_y2 + margin
                    else:
                        if d_y > c_y:
                            d_y = obs_y1 - margin
                        else:
                            d_x = obs_x2 + margin
                    return [d_x, d_y], True
                else:
                    if a == 0:
                        # two robots horizontally aligned
                        if obs_y1 <= c_y <= obs_y2 and point_inside_line(obs_x1, c_y, c_x, c_y, d_x, d_y):
                            return True
                    else:
                        x1 = (obs_y1 - b) / a
                        if obs_x1 <= x1 <= obs_x2 and point_inside_line(x1, obs_y1, c_x, c_y, d_x, d_y):
                            if d_x > obs_x2 or c_y > obs_x2:
                                if d_y > c_y:
                                    d_y = obs_y1 - margin
                                else:
                                    d_x = obs_x2 + margin
                            else:
                                if d_y > c_y:
                                    d_y = obs_y1 - margin
                                else:
                                    d_x = obs_x1 - margin
                            
                            return [d_x, d_y], True
                        else:
                            x2 = (obs_y2 - b) / a
                            if obs_x1 <= x2 <= obs_x2 and point_inside_line(x2, obs_y2, c_x, c_y, d_x, d_y):
                                if d_x > obs_x2 or c_y > obs_x2:
                                    if d_y > c_y:
                                        d_x = obs_x2 + margin
                                    else:
                                        d_y = obs_y2 + margin
                                else:
                                    if d_y > c_y:
                                        d_x = obs_x1 - margin
                                    else:
                                        d_y = obs_y2 + margin
                                return [d_x, d_y], True
    return [d_x, d_y], False