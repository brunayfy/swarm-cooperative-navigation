from collections import defaultdict
from pathlib import Path

import gudhi
import yaml

from Utils import (ensure_valid_deploy_position,
                   filter_one_simplices_exception,
                   get_deployment_absolute_position, get_deployment_angle,
                   get_graph, get_one_simplex_uncov, is_obstacle_simplex,
                   lazy_dijkstra, Map, FenceSubcomplex, point_inside_line)


class Controller:
    def __init__(self, config_path: str, sim_id: str) -> None:
        with open(Path(__file__).parent / config_path, 'r') as f:
            config = yaml.safe_load(f.read())[sim_id]

        self.map = Map(config['map']['boundary'], config['map']['obstacles'])
        self.entrypoint = config['map']['entrypoint']
        self.robot_radius = config['robot_radius']
        # error in measurement of bearings to neighbors, needed so that it doesn't falsely classify 2-Rips complex
        # with pi/3 angle as obstacle
        self.beta = config['beta']

        self.robots, self.skeleton_path = [], []
        self.fence_subcomplex = None
        self.simplices = {0: [], 1: [], 2: []}
        self.robot_is_obstacle = defaultdict(bool)

        # Deploy first robot, assume entrypoint is a valid position
        self.robots.append(self.entrypoint)
        # Deploy second robot in an angle of pi/3 of the first one TODO: treat cases were there is obstacles
        second_robot_deploy_position = [self.entrypoint[0] + 0.5, self.entrypoint[1] + 1]
        pos, is_obstacle = ensure_valid_deploy_position(self.map, self.entrypoint, second_robot_deploy_position)
        self.robots.append(pos)
        if is_obstacle:
            self.robot_is_obstacle[1] = True

        self._update_simplices()
        self._update_fence_subcomplex()
        self._update_skeleton_path()

    def _update_simplices(self):
        self.simplices = {0: [], 1: [], 2: []}
        rips_complex = gudhi.RipsComplex(points=self.robots, max_edge_length=self.robot_radius + 0.1)
        simplex_tree = rips_complex.create_simplex_tree(max_dimension=2)

        one_simplex_obstructed = []
        two_simplex_to_add = []

        for simplex, _ in simplex_tree.get_filtration():
            size = len(simplex) - 1
            if size == 1 and self._check_if_simplex_is_obstructed(simplex):
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

    def _check_if_simplex_is_obstructed(self, simplex: list[int]):
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
                print("check this case where there is a vertically line(two robots are vertically aligned)!")
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
                            print(
                                "check this case where there is a horizontal line(two robots are horizontally aligned)!")
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

        normal_one_simplices, exception_one_simplices = filter_one_simplices_exception(self.simplices[1])
        for one_simplex in normal_one_simplices:
            i, j = one_simplex
            uncov = get_one_simplex_uncov(self.robots, one_simplex, self.simplices[2])
            if uncov:
                theta_i_j_new, theta_j_i_new = get_deployment_angle(obstacle_simplices, self.robots, one_simplex,
                                                                    normal_one_simplices, uncov, self.beta)
                for theta in theta_i_j_new:
                    self.deployment_positions[i].append(
                        get_deployment_absolute_position(self.robots[i], self.robots[j], theta))
                for theta in theta_j_i_new:
                    self.deployment_positions[j].append(
                        get_deployment_absolute_position(self.robots[j], self.robots[i], theta))
                if is_obstacle_simplex(one_simplex, self.robot_is_obstacle):
                    obstacle_simplices.append(one_simplex)
                elif one_simplex in obstacle_simplices:
                    pass  # Already added simplex to obstacles on get_deployment_angle
                else:
                    frontier_simplices.append(one_simplex)
                    frontier_simplices.append([i])
                    frontier_simplices.append([j])
                    # TODO: Check why adding {i} and {j} separately

        self.fence_subcomplex = FenceSubcomplex(obstacle_simplices, frontier_simplices)

    def _update_skeleton_path(self):
        graph = get_graph(self.simplices[1], self.fence_subcomplex)
        dist, paths = lazy_dijkstra(graph, self.robots.index(self.entrypoint), len(self.robots))
        frontier_robots_indices = list(set(sum(self.fence_subcomplex.frontier_simplices, [])))
        closest_frontier_robot_index = min(frontier_robots_indices, key=lambda d: dist[d])
        self.skeleton_path = paths[closest_frontier_robot_index]

    def _push_robot(self):
        if not self.skeleton_path:
            self.robots.append(self.entrypoint)
            return

        frontier, *follow = self.skeleton_path
        frontier_pos = self.robots[frontier]
        frontier_robot_is_obstacle = self.robot_is_obstacle[frontier]
        current, moved, moved_index = None, None, None

        if self.deployment_positions[frontier]:
            # Deploying robots on valid points. If there is an obstacle it will calculate a new deployment position
            # to simulate robot moving to desired position and hitting an obstacle.
            pos, is_obstacle = ensure_valid_deploy_position(self.map, frontier_pos,
                                                            self.deployment_positions[frontier][0])
            self.robots[frontier] = pos
            if is_obstacle:
                self.robot_is_obstacle[frontier] = True
        else:
            print(
                "The robot chosen by dijkstra doesn't have a deployment angle. Implement filtration on skeleton "
                "construction so that it doesn't choose this paths")

        for robot_index in follow:
            current = self.robots[robot_index]
            if moved is None:
                self.robots[robot_index] = frontier_pos
                self.robot_is_obstacle[robot_index] = frontier_robot_is_obstacle
            else:
                self.robots[robot_index] = moved
                self.robot_is_obstacle[robot_index] = self.robot_is_obstacle[moved_index]
            moved = current
            moved_index = robot_index

        self.robots.append(self.entrypoint)
        self.robot_is_obstacle[len(self.robots) - 1] = self.robot_is_obstacle[moved_index]

    def is_full_covered(self) -> bool:
        return False

    def run_iter(self):
        self._push_robot()
        self._update_simplices()
        self._update_fence_subcomplex()
        self._update_skeleton_path()
