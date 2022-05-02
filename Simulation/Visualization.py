import numpy as np
from matplotlib import pyplot as plt


def init_plot(map: dict):
    plt.ion()
    fig = plt.figure()
    fig.suptitle('Swarm Simulation')
    ax = fig.add_subplot(111)
    ax.grid()
    ax.update_datalim([[0, 0], [10, 10]])
    ax.autoscale_view()
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_xticks(list(range(-50, 50)))
    ax.set_yticks(list(range(-50, 50)))

    robots_scatter = ax.scatter([], [], color=[1., 0.63647424, 0.33815827, 1.], label='robots')
    skeleton_scatter = ax.scatter([], [], color='green', label='robots')

    obstacles = {'x': [], 'y': []}
    for y, row in map.items():
        for x, value in enumerate(row):
            if value == 'o':
                obstacles['x'].append(x)
                obstacles['y'].append(y)
    obstacles_scatter = ax.scatter(obstacles['x'], obstacles['y'], marker='s',
                                   color=[0.28039216, 0.33815827, 0.98516223, 1.], label='obstacles')

    ax.legend()

    return {'skeleton_scatter': skeleton_scatter, 'skeleton_path': [], 'robots_scatter': robots_scatter, 'ax': ax,
            'simplices': [], 'texts': []}


def update_plot(plot, robot_coordinates: list[float], one_simplices: list[list[int]], fence_subcomplex, skeleton_path):

    # Plotting the one-simplices edges
    skeleton_path_simplices = [[p1, p2] for p1, p2 in zip(skeleton_path[:-1], skeleton_path[1:])] if len(
        skeleton_path) > 1 else []
    if plot['simplices']:
        for line in plot['simplices']:
            line.remove()
        plot['simplices'] = []
    for one_simplex in one_simplices:
        x1, y1 = robot_coordinates[one_simplex[0]]
        x2, y2 = robot_coordinates[one_simplex[1]]
        if one_simplex in fence_subcomplex['frontier_simplices']:
            plot['simplices'].append(plot['ax'].plot([x1, x2], [y1, y2], color='blue')[0])
        elif one_simplex in fence_subcomplex['obstacle_simplices']:
            plot['simplices'].append(plot['ax'].plot([x1, x2], [y1, y2], color='red')[0])
        elif one_simplex in skeleton_path_simplices:
            plot['simplices'].append(plot['ax'].plot([x1, x2], [y1, y2], color='green')[0])
        else:
            plot['simplices'].append(plot['ax'].plot([x1, x2], [y1, y2], color='orange')[0])

    # Plotting robots
    robot_x, robot_y = [], []
    for i, (x, y) in enumerate(robot_coordinates):
        if i in skeleton_path:
            continue
        robot_x.append(x)
        robot_y.append(y)
    plot['robots_scatter'].set_offsets(np.c_[robot_x, robot_y])

    robot_x, robot_y = [], []
    for p in skeleton_path:
        robot_x.append(robot_coordinates[p][0])
        robot_y.append(robot_coordinates[p][1])
    plot['skeleton_scatter'].set_offsets(np.c_[robot_x, robot_y])

    # Plotting robots ids
    if plot['texts']:
        for text in plot['texts']:
            text.remove()
            plot['texts'] = []
    for i in range(len(robot_coordinates)):
        plot['texts'].append(plt.text(robot_coordinates[i][0], robot_coordinates[i][1], str(i)))

    plt.pause(0.01)
