from collections import defaultdict

import gudhi

from Utils import (ensure_valid_deploy_position,
                   filter_one_simplices_exception,
                   get_deployment_absolute_position, get_deployment_angle,
                   get_graph, get_one_simplex_uncov, is_obstacle_simplex,
                   lazy_dijkstra)


def deploy_first_robots(sim_map: dict, robots: list, entrypoint: list, robots_radius: float):
    robots.append(entrypoint)    
    second_robot_deploy_position = [entrypoint[0], entrypoint[1] + robots_radius]
    dx, dy = ensure_valid_deploy_position(sim_map, entrypoint, second_robot_deploy_position)
    robots.append([dx, dy])


def get_simplices(robots: list, max_edge_length: float) -> dict:
    simplices = {0: [], 1: [], 2: []}
    rips_complex = gudhi.RipsComplex(points=robots, max_edge_length=max_edge_length + 0.1)
    simplex_tree = rips_complex.create_simplex_tree(max_dimension=2)

    for simplex, _ in simplex_tree.get_filtration():
        simplices[len(simplex) - 1].append(simplex)

    return simplices


def get_fence_subcomplex(map_: dict, robots: list, simplices: dict, robots_radius: float):
    """
        1. Filter 1 simplices (exception set)
        2. For one simplex, compute theta sign for all two simplices neighbors
           If all angles have the same sign -> the one simplex belongs to the fence subcomplex
        3. Classify the fence simplices in obstacle or frontier set
    """

    obstacle_simplices, frontier_simplices = [], []
    normal_one_simplices, exception_one_simplices = filter_one_simplices_exception(simplices[1])
    deployment_positions = defaultdict(list)

    for one_simplex in normal_one_simplices:
        i, j = one_simplex
        uncov = get_one_simplex_uncov(robots, one_simplex, simplices[2])
        if uncov:
            theta_i_j_new, theta_j_i_new = get_deployment_angle(obstacle_simplices, robots, one_simplex,
                                                                normal_one_simplices, uncov)
            for theta in theta_i_j_new:
                deployment_positions[i].append(get_deployment_absolute_position(robots[i], robots[j], theta))
            for theta in theta_j_i_new:
                deployment_positions[j].append(get_deployment_absolute_position(robots[j], robots[i], theta))

            if one_simplex in obstacle_simplices:
                pass  # Already added simplex to obstacles on get_deployment_angle
            else:
                frontier_simplices.append(one_simplex)
                frontier_simplices.append([i])
                frontier_simplices.append([j])
                # TODO: Check why adding {i} and {j} separately

    return {
        'obstacle_simplices': obstacle_simplices,
        'frontier_simplices': frontier_simplices,
        'deployment_positions': deployment_positions
    }


def get_skeleton_path(one_simplices, fence_subcomplex, robots, root):
    if root not in robots:
        return []
    graph = get_graph(one_simplices, fence_subcomplex)
    dist, paths = lazy_dijkstra(graph, robots.index(root), len(robots))
    frontier_robots_indices = list(set(sum(fence_subcomplex['frontier_simplices'], [])))
    closest_frontier_robot_index = min(frontier_robots_indices, key=lambda d: dist[d])
    closest_path = paths[closest_frontier_robot_index]
    return closest_path


def push_robot(sim_map: dict, skeleton_path: list, fence_subcomplex, deployment_positions: dict, robots: list, entrypoint: list):
    if not skeleton_path:
        robots.append(entrypoint)
        return

    frontier, *follow = skeleton_path
    frontier_pos = robots[frontier]
    if deployment_positions[frontier]:
        pos, is_obstacle = ensure_valid_deploy_position(sim_map, frontier_pos, deployment_positions[frontier][0])
        robots[frontier] = pos
        if is_obstacle: fence_subcomplex['obstacle_simplices'].append([frontier])
    else:
        print(
            "The robot chosen by dijkstra doesn't have a deployment angle. Implement filtration so that it doesn't "
            "choose this paths")

    current, moved = None, None
    for robot_index in follow:
        current = robots[robot_index]
        if moved is None:
            robots[robot_index] = frontier_pos
        else:
            robots[robot_index] = moved
        moved = current

    robots.append(entrypoint)
