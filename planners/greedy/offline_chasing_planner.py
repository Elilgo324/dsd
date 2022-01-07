from typing import Tuple, Dict

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class OfflineChasingPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float]:
        robots = env.robots
        agents = env.agents

        _, Y_SIZE = env.border

        distances = [[] for _ in range(len(robots))]
        for i in range(len(robots)):
            distances[i] = [robots[i].loc.distance_to(agents[j].loc) for j in range(len(agents))]

        optimal_assignment = linear_sum_assignment(distances)

        movement = {robot: [] for robot in robots}

        for i in range(len(optimal_assignment[0])):
            assigned_agent = agents[optimal_assignment[1][i]]
            movement[robots[optimal_assignment[0][i]]].append(assigned_agent.loc)
            movement[robots[optimal_assignment[0][i]]].append(Point(assigned_agent.x, Y_SIZE))

        return movement, -1

    def __str__(self):
        return 'OfflineChasingPlanner'
