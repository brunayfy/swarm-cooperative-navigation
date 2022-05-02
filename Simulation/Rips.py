from Controller import (get_fence_subcomplex, get_simplices,
                        get_skeleton_path, push_robot)
from Visualization import init_plot, update_plot
import math


def main():

    robots_radius = math.sqrt(3)

    entrypoint_coordinate = [0, 2]
    robots_coordinates = []
    # h = 0.866025
    # robots_coordinates = [[0, 0], [1, 0], [2, 0], [0.5, h], [1.5, h], [2.5, h], [0, 2*h], [1, 2*h], [2, 2*h]]

    map_ = {
        9: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        8: ['o', 'o', 'o', 0, 0, 0, 'o', 'o', 'o', 'o'],
        7: ['o', 'o', 'o', 0, 0, 0, 'o', 'o', 'o', 'o'],
        6: ['o', 'o', 'o', 0, 0, 0, 'o', 'o', 'o', 'o'],
        5: [0, 0, 0, 0, 0, 0, 'o', 0, 0, 'o'],
        4: [0, 0, 0, 0, 0, 0, 'o', 0, 0, 'o'],
        3: [0, 0, 0, 0, 0, 0, 'o', 0, 0, 'o'],
        2: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        1: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        0: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    }

    plot = init_plot(map_)
    while True:
        simplices = get_simplices(robots_coordinates, max_edge_length=robots_radius)
        fence_subcomplex = get_fence_subcomplex(map_, robots_coordinates, simplices, robots_radius)
        skeleton_path = get_skeleton_path(simplices[1], fence_subcomplex, robots_coordinates, entrypoint_coordinate)
        update_plot(plot, robots_coordinates, simplices[1], fence_subcomplex, skeleton_path)
        push_robot(skeleton_path, fence_subcomplex['deployment_positions'], robots_coordinates, entrypoint_coordinate)


if __name__ == '__main__':
    main()
