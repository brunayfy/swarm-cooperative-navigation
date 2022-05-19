import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle


class Plot:
    def __init__(self, controller):
        plt.ion()
        fig = plt.figure()
        fig.suptitle('Swarm Simulation')
        ax = fig.add_subplot(111)
        ax.grid()
        (m_x1, m_y1), (m_x2, m_y2) = controller.map.boundary
        ax.update_datalim([[m_x1 - 0.5, m_y1 - 0.5], [m_x2 + 0.5, m_y2 + 0.5]])
        ax.autoscale_view()
        ax.set_xlabel('x')
        ax.set_ylabel('y')

        # draw map
        ax.add_patch(Rectangle((m_x1, m_y1),
                               m_x2 - m_x1, m_y2 - m_y1,
                               fc='none',
                               ec='black',
                               lw=5))

        # draw obstacles
        for (o_x1, o_y1), (o_x2, o_y2) in controller.map.obstacles:
            rect = matplotlib.patches.Rectangle((o_x1, o_y1),
                                                o_x2 - o_x1, o_y2 - o_y1,
                                                color='grey')
            ax.add_patch(rect)

        ax.legend()
        
        self.ax = ax
        self.robots_scatter = ax.scatter([], [], color=[1., 0.63647424, 0.33815827, 1.], label='robots')
        self.robots_obstacle_scatter = ax.scatter([], [], color='red', label='robots in contact with obstacles')
        self.skeleton_scatter = ax.scatter([], [], color='green', label='skeleton path')
        self.controller = controller
        self.simplices, self.texts = [], []

        self.update_plot()

    def update_plot(self):
        # plotting the one-simplices edges
        skeleton_path_simplices = [[p1, p2] for p1, p2 in
                                   zip(self.controller.skeleton_path[:-1], self.controller.skeleton_path[1:])] \
                                    if len(self.controller.skeleton_path) > 1 else []
        if self.simplices:
            for line in self.simplices:
                line.remove()
            self.simplices = []
        for one_simplex in self.controller.simplices[1]:
            x1, y1 = self.controller.robots[one_simplex[0]]
            x2, y2 = self.controller.robots[one_simplex[1]]
            # TODO: Make the 1 simplex and 2 simplex as tuples so that [0,2] = [2,0]
            if one_simplex in self.controller.fence_subcomplex.frontier_simplices:
                self.simplices.append(self.ax.plot([x1, x2], [y1, y2], color='blue')[0])
            elif one_simplex in self.controller.fence_subcomplex.obstacle_simplices:
                self.simplices.append(self.ax.plot([x1, x2], [y1, y2], color='red')[0])
            elif one_simplex in skeleton_path_simplices:
                self.simplices.append(self.ax.plot([x1, x2], [y1, y2], color='green')[0])
            else:
                self.simplices.append(self.ax.plot([x1, x2], [y1, y2], color='orange')[0])

        # plotting robots
        robot_x, robot_y = [], []
        for i, (x, y) in enumerate(self.controller.robots):
            if i in self.controller.skeleton_path or self.controller.robot_is_obstacle[i]:
                continue
            robot_x.append(x)
            robot_y.append(y)
        self.robots_scatter.set_offsets(np.c_[robot_x, robot_y])

        robot_x, robot_y = [], []
        for p in self.controller.skeleton_path:
            robot_x.append(self.controller.robots[p][0])
            robot_y.append(self.controller.robots[p][1])
        self.skeleton_scatter.set_offsets(np.c_[robot_x, robot_y])

        # robots that are in contact with obstacle: red
        robot_x, robot_y = [], []
        for p in [index for index, status in self.controller.robot_is_obstacle.items() if status]:
            robot_x.append(self.controller.robots[p][0])
            robot_y.append(self.controller.robots[p][1])
        self.robots_obstacle_scatter.set_offsets(np.c_[robot_x, robot_y])

        # plotting robots ids
        if self.texts:
            for text in self.texts:
                text.remove()
                self.texts = []
        for i in range(len(self.controller.robots)):
            self.texts.append(plt.text(self.controller.robots[i][0], self.controller.robots[i][1], str(i)))

        plt.pause(1)
