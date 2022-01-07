from typing import Dict, Tuple

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class OnlineChasingPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float]:
        robots = env.robots
        agents = env.agents

        distances = [[] for _ in range(len(robots))]
        for i in range(len(robots)):
            distances[i] = [robots[i].loc.distance_to(agents[j].loc) for j in range(len(agents))]

        optimal_assignment = linear_sum_assignment(distances)

        movement = {robot: [] for robot in robots}

        for i in range(len(optimal_assignment[0])):
            movement[robots[optimal_assignment[0][i]]].append(agents[optimal_assignment[1][i]])

        return movement, -1

    def __str__(self):
        return 'OnlineChasingPlanner'
