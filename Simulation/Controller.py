from pathlib import Path
from collections import defaultdict

import gudhi
import yaml

from Utils import (ensure_valid_deploy_position,
                   filter_one_simplices_exception,
                   get_deployment_absolute_position, get_deployment_angle,
                   get_graph, get_one_simplex_uncov, is_obstacle_simplex,
                   lazy_dijkstra)


class Controller:
    def __init__(self, config_path: str, sim_id: str) -> None:
        with open(Path(__file__).parent / config_path, 'r') as f:
            config = yaml.safe_load(f.read())[sim_id]
        
        self.map = config['map']
        self.entrypoint = self.map['entrypoint']
        self.robot_radius = config['robot_radius']
        self.robots, self.skeleton_path, self.fence_subcomplex = [], [], {}
        self.simplices = {0: [], 1: [], 2: []}
        self.robot_is_obstacle = defaultdict(bool)

        # Deploy first robots
        self.robots.append(self.entrypoint)
        second_robot_deploy_position = [self.entrypoint[0], self.entrypoint[1] + self.robot_radius]
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

        for simplex, _ in simplex_tree.get_filtration():
            self.simplices[len(simplex) - 1].append(simplex)


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
                                                                    normal_one_simplices, uncov)
                for theta in theta_i_j_new:
                    self.deployment_positions[i].append(get_deployment_absolute_position(self.robots[i], self.robots[j], theta))
                for theta in theta_j_i_new:
                    self.deployment_positions[j].append(get_deployment_absolute_position(self.robots[j], self.robots[i], theta))
                if is_obstacle_simplex(one_simplex, self.robot_is_obstacle):
                    obstacle_simplices.append(one_simplex)
                elif one_simplex in obstacle_simplices:
                    pass  # Already added simplex to obstacles on get_deployment_angle
                else:
                    frontier_simplices.append(one_simplex)
                    frontier_simplices.append([i])
                    frontier_simplices.append([j])
                    # TODO: Check why adding {i} and {j} separately

        self.fence_subcomplex = {
            'obstacle_simplices'   : obstacle_simplices,
            'frontier_simplices'   : frontier_simplices,
            'deployment_positions' : self.deployment_positions
        }


    def _update_skeleton_path(self):     
        graph = get_graph(self.simplices[1], self.fence_subcomplex)
        dist, paths = lazy_dijkstra(graph, self.robots.index(self.entrypoint), len(self.robots))
        frontier_robots_indices = list(set(sum(self.fence_subcomplex['frontier_simplices'], [])))
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
            pos, is_obstacle = ensure_valid_deploy_position(self.map, frontier_pos, self.deployment_positions[frontier][0])
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
        self.robot_is_obstacle[len(self.robots)-1] = self.robot_is_obstacle[moved_index]


    def is_full_covered(self) -> bool:
        return False

    def run_iter(self):
        self._push_robot()
        self._update_simplices()
        self._update_fence_subcomplex()
        self._update_skeleton_path()
