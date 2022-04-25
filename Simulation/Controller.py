import math

import gudhi
import numpy as np

from Utils import (get_angle, get_graph, is_in_contact_with_obstacle,
                   lazy_dijkstra,
                   no_visible_robots_in_activated_touch_sensor_angle)


def get_simplices(robots: list[list], max_edge_length: float = math.sqrt(2)) -> dict:
    simplices = {0: [], 1: [], 2: []}
    rips_complex = gudhi.RipsComplex(points=robots, max_edge_length=max_edge_length)
    simplex_tree = rips_complex.create_simplex_tree(max_dimension=2)

    for simplex, _ in simplex_tree.get_filtration():
        simplices[len(simplex) - 1].append(simplex)

    return simplices


def get_fence_subcomplex(map: list[list], robots: list[list], simplices: list[list]):
    """
        1. Filter 1 simplices (exception set)
        2. For one simplex, compute theta sign for all two simplices neighbors
           If all angles have the same sign -> the one simplex belongs to the fence subcomplex
        3. Classify the fence simplices in obstacle or frontier set
    """

    def filter_one_simplices_exception(one_simplices: list[list]) -> dict:
        # TODO: To be implemented later
        return one_simplices, []

    def get_one_simplex_uncov(one_simplex: list, two_simplices: list[list]) -> list:
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

    def get_deployment_angle(robots: list, one_simplex: list, one_simplices: list, uncov: list) -> float:
        theta_i_j_new, theta_j_i_new = [], []

        i, j = one_simplex
        for sigma in uncov:
            S_i, S_j = get_closest_fence_candidates(robots, one_simplex, one_simplices, sigma)
            if S_i == []:
                theta_i_j_new.append(sigma * 60)
            else:
                k_i = min([[k, abs(get_angle(robots[i], robots[j], robots[k]))] for k in S_i],
                          key=lambda x: x[1])[0]
                theta_i_j_k_i = get_angle(robots[i], robots[j], robots[k_i])
                if abs(theta_i_j_k_i) < 60:
                    obstacle_simplices.append([i, k_i])  # TODO: Check this!
                else:
                    theta_i_j_new.append(sigma * min([60, abs(get_angle(robots[i], robots[j], robots[k_i]) / 2)]))

            if S_j == []:
                theta_j_i_new.append(-sigma * 60)
            else:
                k_j = min([[k, abs(get_angle(robots[j], robots[i], robots[k]))] for k in S_j],
                          key=lambda x: x[1])[0]
                theta_j_i_k_j = get_angle(robots[i], robots[j], robots[k_j])

                if abs(theta_j_i_k_j) < 60:
                    obstacle_simplices.append([i, k_j])  # TODO: Check this!
                else:
                    theta_j_i_new.append(-sigma * min([60, abs(get_angle(robots[j], robots[i], robots[k_j]) / 2)]))

        return [theta_i_j_new, theta_j_i_new]

    def is_obstacle_simplex(robots, map, one_simplex) -> bool:
        i, j = one_simplex
        if (
                is_in_contact_with_obstacle(map, robots[i]) and
                is_in_contact_with_obstacle(map, robots[j]) and
                no_visible_robots_in_activated_touch_sensor_angle(robots, robots[i]) and
                no_visible_robots_in_activated_touch_sensor_angle(robots, robots[j])
        ):
            return True
        return False

    obstacle_simplices, frontier_simplices = [], []
    normal_one_simplices, exception_one_simplices = filter_one_simplices_exception(simplices[1])
    for one_simplex in normal_one_simplices:
        uncov = get_one_simplex_uncov(one_simplex, simplices[2])
        if uncov != []:
            deployment_angle = get_deployment_angle(robots, one_simplex, normal_one_simplices, uncov)
            if is_obstacle_simplex(robots, map, one_simplex):  # TODO: Check obstacle/frontier order
                obstacle_simplices.append(one_simplex)
                i, j = one_simplex
                obstacle_simplices.append([i])
                obstacle_simplices.append([j])
            else:
                frontier_simplices.append(one_simplex)

    return {
        'obstacle_simplices': obstacle_simplices,
        'frontier_simplices': frontier_simplices
    }


def get_skeleton_path(one_simplices, fence_subcomplex, robots_coordinates, root):
    if one_simplices == []:
        return []

    graph = get_graph(one_simplices, fence_subcomplex)
    dist, paths = lazy_dijkstra(graph, robots_coordinates.index(root))
    frontier_robots_indices = list(set(sum(fence_subcomplex['frontier_simplices'], [])))
    closest_frontier_robot_index = min(frontier_robots_indices, key=lambda d: dist[d])
    closest_path = paths[closest_frontier_robot_index]
    return closest_path


def push_robot(skeleton_path: list, robots_coordinates: list, entrypoint_coordinate: list):
    if skeleton_path == []:
        robots_coordinates.append(entrypoint_coordinate)
        return

    frontier, *follow = skeleton_path

    # TODO: Replace by position calculated by deployment angle
    deployment_position = [robots_coordinates[frontier][0] + 1, robots_coordinates[frontier][1]]
    robots_coordinates[frontier] = deployment_position

    current, moved = None, None
    for robot_index in follow:
        current = robots_coordinates[robot_index]
        if moved is None:
            robots_coordinates[robot_index] = frontier
        else:
            robots_coordinates[robot_index] = moved
        moved = current

    robots_coordinates.append(entrypoint_coordinate)
