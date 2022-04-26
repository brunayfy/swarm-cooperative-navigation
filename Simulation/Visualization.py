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
    ax.set_xticks(list(range(-500, 500)))
    ax.set_yticks(list(range(-500, 500)))

    robots_scatter = ax.scatter([], [], color=[1., 0.63647424, 0.33815827, 1.], label='robots')

    obstacles = {'x': [], 'y': []}
    for y, row in map.items():
        for x, value in enumerate(row):
            if value == 'o':
                obstacles['x'].append(x)
                obstacles['y'].append(y)
    obstacles_scatter = ax.scatter(obstacles['x'], obstacles['y'], marker='s',
                                   color=[0.28039216, 0.33815827, 0.98516223, 1.], label='obstacles')

    ax.legend()

    return {'robots_scatter': robots_scatter, 'ax': ax, 'simplices': [], 'texts': []}


def update_plot(plot, robot_coordinates: list[float], one_simplices: list[list[int]]):
    # Plotting robots
    robot_x, robot_y = [], []
    for x, y in robot_coordinates:
        robot_x.append(x)
        robot_y.append(y)
    plot['robots_scatter'].set_offsets(np.c_[robot_x, robot_y])
    
    # Adding the id for the robots
    if plot['texts']:
        for text in plot['texts']:
            text.remove()
            plot['texts'] = []
    for i in range(len(robot_coordinates)):
        plot['texts'].append(plt.text(robot_x[i], robot_y[i], str(i)))

    # Plotting the one-simplices edges
    if plot['simplices']:
        for line in plot['simplices']:
            line.remove()
        plot['simplices'] = []
    for one_simplex in one_simplices:
        x1, y1 = robot_coordinates[one_simplex[0]]
        x2, y2 = robot_coordinates[one_simplex[1]]
        plot['simplices'].append(plot['ax'].plot([x1, x2], [y1, y2], color='orange')[0])

    #plt.pause(1)
