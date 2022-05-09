import math
from collections import defaultdict

from Controller import (get_fence_subcomplex, get_simplices, get_skeleton_path,
                        push_robot, deploy_first_robots)
from Visualization import init_plot, update_plot


def main():
    robots_radius = math.sqrt(2)
    entrypoint = [1, 1]
    robots = []
    robot_is_obstacle = defaultdict(bool)
    # h = 0.866025
    # robots = [[0, 0], [1, 0], [2, 0], [0.5, h], [1.5, h], [2.5, h], [0, 2*h], [1, 2*h], [2, 2*h]]

    sim_map = {
        "boundary": [[0, 0], [20, 20]],
        "obstacles": [[[2, 2], [4, 4]], [[5, 5], [7, 8]]]
    }
    plot = init_plot(sim_map)
    deploy_first_robots(sim_map, robots, entrypoint, robots_radius, robot_is_obstacle)

    while True:
        simplices = get_simplices(robots, max_edge_length=robots_radius)
        fence_subcomplex = get_fence_subcomplex(robots, simplices, robot_is_obstacle)
        skeleton_path = get_skeleton_path(simplices[1], fence_subcomplex, robots, entrypoint)
        update_plot(plot, robots, simplices[1], fence_subcomplex, skeleton_path, robot_is_obstacle)
        push_robot(sim_map, skeleton_path, fence_subcomplex['deployment_positions'], robots, entrypoint,
                   robot_is_obstacle)


if __name__ == '__main__':
    main()
