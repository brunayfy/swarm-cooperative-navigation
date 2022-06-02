import math
from collections import defaultdict
from pathlib import Path

import gudhi
import yaml

from Utils import (FenceSubcomplex, Map, distance,
                   ensure_valid_deploy_position,
                   filter_one_simplices_exception,
                   get_deployment_absolute_position, get_deployment_angle,
                   get_graph, get_one_simplex_uncov, get_pair_combinations,
                   is_obstacle_simplex, lazy_dijkstra, point_inside_line)


class Controller:
    def __init__(self, config_path: str, sim_id: str) -> None:
        with open(Path(__file__).parent / config_path, 'r') as f:
            config = yaml.safe_load(f.read())[sim_id]
        self.is_full_covered = False
        self.map = Map(config['map']['boundary'], config['map']['obstacles'])
        self.entrypoint = config['map']['entrypoint']
        self.robot_radius = config['robot_radius']
        # error in measurement of bearings to neighbors, needed so that it doesn't falsely classify 2-Rips complex
        # with pi/3 angle as obstacle
        self.beta = config['beta']

        self.robots, self.skeleton_path = [], []
        self.fence_subcomplex = None
        self.simplices = {0: [], 1: [], 2: []}
        self.normal_one_simplices, self.exception_one_simplices = [], []

        self.robot_is_obstacle = defaultdict(bool)

        # Deploy first robot, assuming entrypoint is a valid position!
        self.robots.append(self.entrypoint)
        # Deploy second robot in an angle of pi/3 of the first one
        second_robot_deploy_position = [self.entrypoint[0] + self.robot_radius / 2,
                                        self.entrypoint[1] + math.sin(math.pi / 3) * self.robot_radius]
        pos, is_obstacle = ensure_valid_deploy_position(self.map, self.entrypoint, second_robot_deploy_position)
        self.robots.append(pos)
        if is_obstacle:
            self.robot_is_obstacle[1] = True

        self._update_simplices()
        self._update_fence_subcomplex()
        self._update_skeleton_path()

    def _update_simplices(self):
        def _get_angle(coord1: list[float, float], coord2: list[float, float]):
            x1, y1 = coord1
            x2, y2 = coord2
            return math.atan2(y2 - y1, x2 - x1)
        
        self.simplices = {0: [], 1: [], 2: []}
        rips_complex = gudhi.RipsComplex(points=self.robots, max_edge_length=self.robot_radius + 0.05)
        simplex_tree = rips_complex.create_simplex_tree(max_dimension=2)

        one_simplex_obstructed = []
        two_simplex_to_add = []

        one_simplices = [simplex for simplex, _ in simplex_tree.get_filtration() if len(simplex) - 1 == 1]
        neighboors = defaultdict(dict)
        for i, j in one_simplices:
            neighboors[i][j] = {
                'angle'     : _get_angle(self.robots[i], self.robots[j]),
                'overlap'   : False
            }
        
        for i, i_neighboors in neighboors.items():
            for n1, n2 in get_pair_combinations(list(i_neighboors.keys())):
                if neighboors[i][n1]['angle'] == neighboors[i][n2]['angle']:
                    if distance(self.robots[i], self.robots[n1]) > distance(self.robots[i], self.robots[n2]):
                        neighboors[i][n1]['overlap'] = True
                    else:
                        neighboors[i][n2]['overlap'] = True

        for simplex, _ in simplex_tree.get_filtration():
            size = len(simplex) - 1
            if size == 1 and self._check_if_simplex_is_obstructed(simplex, neighboors):
                one_simplex_obstructed.append(simplex)
            elif size == 2:
                two_simplex_to_add.append(simplex)
            else:
                self.simplices[size].append(simplex)

        # Separately adding two_simplex that doesn't have obstruction
        if one_simplex_obstructed:
            for two_simplex in two_simplex_to_add:
                for one_simplex in one_simplex_obstructed:
                    if len([x for x in two_simplex if x not in one_simplex]) != 1:
                        self.simplices[2].append(two_simplex)
        else:
            self.simplices[2] = two_simplex_to_add

    def _check_if_simplex_is_obstructed(self, simplex: list[int], neighboors: list):
        # Check if one-simplices overlap
        i, j = simplex
        if neighboors[i][j]['overlap'] == True:
            return True
        
        # Check if one-simplices cross an obstacle(robots cannot see each other)
        robot_1, robot_2 = simplex

        robot_x1, robot_y1 = self.robots[robot_1]
        robot_x2, robot_y2 = self.robots[robot_2]

        # Finding line equation : y = ax + b
        a_den = robot_x2 - robot_x1
        if a_den != 0:
            a = (robot_y2 - robot_y1) / a_den
            b = robot_y1 - a * robot_x1

        for (obs_x1, obs_y1), (obs_x2, obs_y2) in self.map.obstacles:
            if a_den == 0:
                # two robots are vertically aligned
                if obs_x1 <= robot_x1 <= obs_x2 and point_inside_line(robot_x1, obs_y1, robot_x1, robot_y1,
                                                                      robot_x2, robot_y2):
                    return True
            else:
                y1 = a * obs_x1 + b
                if obs_y1 <= y1 <= obs_y2 and point_inside_line(obs_x1, y1, robot_x1, robot_y1, robot_x2,
                                                                robot_y2):
                    return True
                else:
                    y2 = a * obs_x2 + b
                    if obs_y1 <= y2 <= obs_y2 and point_inside_line(obs_x2, y2, robot_x1, robot_y1, robot_x2,
                                                                    robot_y2):
                        return True
                    else:
                        if a == 0:
                            # two robots horizontally aligned
                            if obs_y1 <= robot_y1 <= obs_y2 and point_inside_line(obs_x1, robot_y1, robot_x1,
                                                                                  robot_y1, robot_x2, robot_y2):
                                return True
                        else:
                            x1 = (obs_y1 - b) / a
                            if obs_x1 <= x1 <= obs_x2 and point_inside_line(x1, obs_y1, robot_x1, robot_y1,
                                                                            robot_x2, robot_y2):
                                return True
                            else:
                                x2 = (obs_y2 - b) / a
                                if obs_x1 <= x2 <= obs_x2 and point_inside_line(x2, obs_y2, robot_x1, robot_y1,
                                                                                robot_x2, robot_y2):
                                    return True
        return False

    def _update_fence_subcomplex(self):
        """
            1. Filter 1 simplices (exception set)
            2. For one simplex, compute theta sign for all two simplices neighbors
            If all angles have the same sign -> the one simplex belongs to the fence subcomplex
            3. Classify the fence simplices in obstacle or frontier set
        """
        obstacle_simplices, frontier_simplices = [], []
        self.deployment_positions = defaultdict(list)

        normal_one_simplices, exception_one_simplices = filter_one_simplices_exception(self.robots, self.simplices[2], self.simplices[1])
        self.normal_one_simplices = normal_one_simplices
        self.exception_one_simplices = exception_one_simplices

        for one_simplex in normal_one_simplices:
            i, j = one_simplex
            uncov = get_one_simplex_uncov(self.robots, one_simplex, self.simplices[2])
            if uncov:
                theta_i_j_new, theta_j_i_new = get_deployment_angle(obstacle_simplices, self.robots, one_simplex,
                                                                    normal_one_simplices, uncov, self.beta)

                if is_obstacle_simplex(one_simplex, self.robot_is_obstacle):
                    obstacle_simplices.append(one_simplex)
                elif one_simplex in obstacle_simplices:
                    pass  # Already added simplex to obstacles on get_deployment_angle
                else:
                    frontier_simplices.append(one_simplex)

                    # Only appending deployment positions for frontier robots
                    for theta in theta_i_j_new:
                        pos = get_deployment_absolute_position(self.robots[i], self.robots[j], theta)
                        self.deployment_positions[i].append(pos)
                        self.deployment_positions[j].append(pos)

                    # May be used when doing decentralized processing
                    # for theta in theta_i_j_new:
                    #     self.deployment_positions[i].append(
                    #         get_deployment_absolute_position(self.robots[i], self.robots[j], theta))
                    # for theta in theta_j_i_new:
                    #     self.deployment_positions[j].append(
                    #         get_deployment_absolute_position(self.robots[j], self.robots[i], theta))
                    # NOTE: Didn't add {i} and {j} separately as it's not used to calculate skeleton path

        self.fence_subcomplex = FenceSubcomplex(obstacle_simplices, frontier_simplices)

    def _update_skeleton_path(self):
        graph = get_graph(self.simplices[1], self.fence_subcomplex, self.robot_is_obstacle)
        dist, paths = lazy_dijkstra(graph, self.robots.index(self.entrypoint), len(self.robots))
        frontier_robots_indices = list(set(sum(self.fence_subcomplex.frontier_simplices, [])))
        if not frontier_robots_indices:
            self.is_full_covered = True
            print("Exploration completed!")
            return
        closest_frontier_robot_index = min(frontier_robots_indices, key=lambda d: dist[d])
        a = paths[closest_frontier_robot_index]
        #TODO: check a === [5,8,9] it forms a simplex where it crosses another
        self.skeleton_path = paths[closest_frontier_robot_index]

    def _push_robot(self):
        if not self.skeleton_path:
            self.robots.append(self.entrypoint)
            return

        self.robots.append(self.entrypoint)
        self.skeleton_path.append(len(self.robots) - 1)
        reversed_skeleton_path = list(reversed(self.skeleton_path))
        for i, robot in enumerate(reversed_skeleton_path):
            if i == len(self.skeleton_path) - 1:
                if self.deployment_positions[robot]:
                    # Deploying robots on valid points. If there is an obstacle it will calculate a new deployment
                    # position to simulate robot moving to desired position and hitting an obstacle.
                    pos, is_obstacle = ensure_valid_deploy_position(self.map, self.robots[robot],
                                                                    self.deployment_positions[robot][0])
                    self.robots[robot] = pos
                    if is_obstacle:
                        self.robot_is_obstacle[robot] = True
                else:
                    print("The robot chosen by dijkstra doesn't have a deployment angle. Implement filtration on "
                          "skeleton construction so that it doesn't choose this paths")
            else:
                next_robot = reversed_skeleton_path[i + 1]
                self.robots[robot] = self.robots[next_robot]
                self.robot_is_obstacle[robot] = self.robot_is_obstacle[next_robot]

    def run_iter(self):
        self._push_robot()
        self._update_simplices()
        # if __debug__:  # This plot is partial with fence complex not updated used for testing, use -O to remove debug
            # self.plot.update_plot()
        self._update_fence_subcomplex()
        self._update_skeleton_path()
