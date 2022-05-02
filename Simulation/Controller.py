from collections import defaultdict

import gudhi

from Utils import is_obstacle_simplex, get_deployment_absolute_position, get_deployment_angle, \
    get_one_simplex_uncov, filter_one_simplices_exception, get_graph, lazy_dijkstra


def get_simplices(robots: list[list], max_edge_length: float) -> dict:
    simplices = {0: [], 1: [], 2: []}
    rips_complex = gudhi.RipsComplex(points=robots, max_edge_length=max_edge_length + 0.1)
    simplex_tree = rips_complex.create_simplex_tree(max_dimension=2)

    for simplex, _ in simplex_tree.get_filtration():
        simplices[len(simplex) - 1].append(simplex)

    return simplices


def get_fence_subcomplex(map_: dict, robots: list[list], simplices: dict, robots_radius: float):
    """
        1. Filter 1 simplices (exception set)
        2. For one simplex, compute theta sign for all two simplices neighbors
           If all angles have the same sign -> the one simplex belongs to the fence subcomplex
        3. Classify the fence simplices in obstacle or frontier set
    """

    obstacle_simplices, frontier_simplices = [], []
    normal_one_simplices, exception_one_simplices = filter_one_simplices_exception(simplices[1])
    deployment_positions = defaultdict(list)

    # Robots that don't have any other robot near
    one_simplices_robots = list(set(sum(simplices[1], [])))
    zero_simplices_robots = sum(simplices[0], [])
    diff = [[x] for x in zero_simplices_robots if x not in one_simplices_robots]

    frontier_simplices += diff
    for lonely_robot in diff:
        # TODO: Set deployment position based on near obstacles and considering sensor radius
        x, y = robots[lonely_robot[0]]
        deployment_positions[lonely_robot[0]] = [[x, y + robots_radius]]

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

            if is_obstacle_simplex(robots, map_, one_simplex):  # TODO: Check obstacle/frontier order
                obstacle_simplices.append(one_simplex)
                i, j = one_simplex
                # TODO: Check why adding {i} and {j} separately
                obstacle_simplices.append([i])
                obstacle_simplices.append([j])
            elif one_simplex in obstacle_simplices:
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


def get_skeleton_path(one_simplices, fence_subcomplex, robots_coordinates, root):
    if root not in robots_coordinates:
        return []
    graph = get_graph(one_simplices, fence_subcomplex)
    dist, paths = lazy_dijkstra(graph, robots_coordinates.index(root), len(robots_coordinates))
    frontier_robots_indices = list(set(sum(fence_subcomplex['frontier_simplices'], [])))
    closest_frontier_robot_index = min(frontier_robots_indices, key=lambda d: dist[d])
    closest_path = paths[closest_frontier_robot_index]
    return closest_path


def push_robot(skeleton_path: list, deployment_positions: dict, robots_coordinates: list, entrypoint_coordinate: list):
    if not skeleton_path:
        robots_coordinates.append(entrypoint_coordinate)
        return

    frontier, *follow = skeleton_path
    frontier_pos = robots_coordinates[frontier]
    if deployment_positions[frontier]:
        robots_coordinates[frontier] = deployment_positions[frontier][0]
    else:
        print(
            "The robot chosen by dijkstra doesn't have a deployment angle. Implement filtration so that it doesn't "
            "choose this paths")

    current, moved = None, None
    for robot_index in follow:
        current = robots_coordinates[robot_index]
        if moved is None:
            robots_coordinates[robot_index] = frontier_pos
        else:
            robots_coordinates[robot_index] = moved
        moved = current

    robots_coordinates.append(entrypoint_coordinate)
