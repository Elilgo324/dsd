from typing import Dict

import numpy as np
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import KMeans

from planners.planner import Planner
from utils.functions import *


class KmeansAssignmentPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents_copy = [a.clone() for a in env.agents]
        movement = {robot: [] for robot in robots}

        X = np.array([[a.x, a.y] for a in agents_copy])
        kmeans = KMeans(n_clusters=len(robots))
        labels = kmeans.fit_predict(X)
        clusters = [[] for _ in range(len(robots))]
        for i in range(len(labels)):
            agent = agents_copy[i]
            label = labels[i]
            clusters[label].append(agent)

        iterative_assignments = {robot: {i_cluster: iterative_assignment([robot], [a.clone() for a in clusters[
            i_cluster]]) for i_cluster in range(len(clusters))} for robot in robots}
        distances = [[iterative_assignments[robot][i_cluster]['damage']
                      for i_cluster in range(len(clusters))] for robot in robots]

        optimal_assignment = linear_sum_assignment(distances)
        assigned_robots = optimal_assignment[0]
        assigned_clusters = optimal_assignment[1]

        damage = 0
        completion_time = 0
        agents_disabled = 0

        for i_robot in assigned_robots:
            robot = robots[i_robot]
            i_cluster = assigned_clusters[i_robot]
            movement[robot] = iterative_assignments[robot][i_cluster]['movement'][robot]

            damage += iterative_assignments[robot][i_cluster]['damage']
            completion_time = max(completion_time, iterative_assignments[robot][i_cluster]['completion_time'])
            agents_disabled += iterative_assignments[robot][i_cluster]['num_disabled']

        return movement, completion_time, damage, agents_disabled

    def __str__(self):
        return 'KmeansAssignmentPlanner'
