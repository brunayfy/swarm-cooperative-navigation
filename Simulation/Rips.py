from Controller import (get_fence_subcomplex, get_simplices,
                        get_skeleton_path, push_robot)
from Visualization import init_plot, update_plot


def main():
    # def get_coordinates_from_map2(map):
    #     return {
    #         'obstacles': [[x, y] for x, row in enumerate(map) for y, value in enumerate(row) if value == 'o'],
    #         'robots': [[x, y] for x, row in enumerate(map) for y, value in enumerate(row) if value == 'r']
    #     }
    # def update_plot2(coordinates):
    #     plt.clf()
    #     plt.xlabel('x')
    #     plt.ylabel('y')
    #     plt.grid()
    #
    #     x = [x for _, x in coordinates['obstacles']]
    #     y = [y for y, _ in coordinates['obstacles']]
    #     plt.scatter(x, y, marker='s')
    #
    #     x = [x for _, x in coordinates['robots']]
    #     y = [y for y, _ in coordinates['robots']]
    #     plt.scatter(x, y, marker='o', )
    #     for i in range(len(x)):
    #         plt.text(x[i], y[i], str(i))
    #     plt.pause(0.01)
    # def push_robots2(path, new_position, map):
    #     print('pushing robots down path:', path)
    #     new_robots_pos = map.robots
    #
    #     # Robots are moving along the path.
    #     for i in range(len(path) - 1):
    #         new_robots_pos[path[i]] = map.robots[path[i + 1]]
    #
    #     # the first position will be occupied by the new robot
    #     new_robots_pos.append(map.robots[path[0]])
    #
    #     # the new position will be occupied by the robot on the tip of the frontier
    #     new_robots_pos[path[-1]] = new_position
    #
    #     return {
    #         'obstacles': map.obstacles,
    #         'robots': new_robots_pos
    #     }
    entrypoint_coordinate = [1, 0]
    h = 0.866025
    robots_coordinates = [[0,0],[1,0],[2,0],[0.5,h],[1.5,h],[2.5,h],[0,2],[1,2],[2,2]]
    map = {
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
    plot = init_plot(map)
    while True:
        simplices = get_simplices(robots_coordinates)
        update_plot(plot, robots_coordinates, simplices[1])
        fence_subcomplex = get_fence_subcomplex(map, robots_coordinates, simplices)
        skeleton_path = get_skeleton_path(simplices[1], fence_subcomplex, robots_coordinates, entrypoint_coordinate)
        # push_robot(skeleton_path, robots_coordinates, entrypoint_coordinate)



if __name__ == '__main__':
    main()
