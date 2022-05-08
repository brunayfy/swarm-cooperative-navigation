import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle


def init_plot(sim_map: dict):
    plt.ion()
    fig = plt.figure()
    fig.suptitle('Swarm Simulation')
    ax = fig.add_subplot(111)
    ax.grid()
    ax.update_datalim([[sim_map['boundary'][0][0] - 5, sim_map['boundary'][0][1] - 5], [sim_map['boundary'][1][0] + 5, sim_map['boundary'][1][1] + 5]])
    ax.autoscale_view()
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # create robots and skeletons scatter
    robots_scatter = ax.scatter([], [], color=[1., 0.63647424, 0.33815827, 1.], label='robots')
    skeleton_scatter = ax.scatter([], [], color='green', label='skeleton path')

    # draw map
    ax.add_patch(Rectangle((sim_map['boundary'][0][0], sim_map['boundary'][0][1]),
                sim_map['boundary'][1][0] - sim_map['boundary'][0][0], sim_map['boundary'][1][1] - sim_map['boundary'][0][1],
                fc ='none', 
                ec ='black',
                lw = 5))

    # draw obstacles
    for obstacle in sim_map['obstacles']:
        rect = matplotlib.patches.Rectangle((obstacle[0][0], obstacle[0][1]),
                                            obstacle[1][0] - obstacle[0][0], obstacle[1][1] - obstacle[0][1],
                                            color ='grey')
        ax.add_patch(rect)

    ax.legend()

    return {
        'skeleton_scatter' : skeleton_scatter, 
        'robots_scatter'   : robots_scatter, 
        'ax'               : ax,
        'skeleton_path'    : [], 
        'simplices'        : [], 
        'texts'            : []
    }


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
