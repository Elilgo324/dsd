import numpy as np
from scipy.optimize import linear_sum_assignment
from sklearn.cluster import DBSCAN

from planners.planner import Planner
from utils.functions import *


class DbscanAssignmentPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, float, int]:
        robots = env.robots
        agents_copy = [a.clone() for a in env.agents]
        movement = {robot: [] for robot in robots}
        v = agents_copy[0].v
        d = robots[0].r

        # create clusters
        X = np.array([[a.x, a.y] for a in agents_copy])
        dbscan = DBSCAN(eps=d)
        labels = dbscan.fit_predict(X)
        clusters = [[] for _ in range(len(robots))]
        for i in range(len(labels)):
            agent = agents_copy[i]
            label = labels[i]
            clusters[label].append(agent)

        # if there are enough robots
        if len(clusters) <= len(robots):
            iterative_assignments = {robot: {i_cluster: iterative_assignment([robot], [a.clone() for a in clusters[
                i_cluster]]) for i_cluster in range(len(clusters))} for robot in robots}
            distances = [[iterative_assignments[robot][i_cluster]['damage']
                          for i_cluster in range(len(clusters))] for robot in robots]

            optimal_assignment = linear_sum_assignment(distances)
            assigned_robots, assigned_agents = map(list, zip(*optimal_assignment))
            optimal_assignment = [assigned_robots, assigned_agents]
            assigned_robots = optimal_assignment[0]
            assigned_clusters = optimal_assignment[1]

            damage = 0
            active_time = 0
            completion_time = 0
            agents_disabled = 0

            for i_robot in assigned_robots:
                robot = robots[i_robot]
                i_cluster = assigned_clusters[i_robot]
                movement[robot] = iterative_assignments[robot][i_cluster]['movement'][robot]

                damage += iterative_assignments[robot][i_cluster]['damage']
                active_time = max(active_time, iterative_assignments[robot][i_cluster]['active_time'])
                completion_time = active_time
                agents_disabled += iterative_assignments[robot][i_cluster]['num_disabled']

            return movement, active_time, completion_time, damage, agents_disabled

        # if at least one robot needs to visit two clusters
        centroids = [Point(sum([a.x for a in c]) / len(c), sum([a.y for a in c]) / len(c)) for c in clusters]
        agents_centroids = [FixedVelocityAgent(loc=centroids[i], v=v) for i in range(len(clusters))]

    def __str__(self):
        return 'DbscanAssignmentPlanner'
