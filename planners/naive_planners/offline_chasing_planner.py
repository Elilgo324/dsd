import random
from typing import Dict, List

from scipy.optimize import linear_sum_assignment

from agents.base_agent import BaseAgent
from planners.planner import Planner
from robots.basic_robot import BasicRobot
from environment import Environment
from utils.functions import *


class OfflineChasingPlanner(Planner):
    def __init__(self, environment: Environment):
        super().__init__(environment)

    def plan(self) -> None:
        robots = self._environment.robots
        agents = self._environment.agents

        distances = [[] for _ in range(len(robots))]
        for i in range(len(robots)):
            distances[i] = [robots[i].loc.distance_to(agents[j].loc) for j in range(len(agents))]

        optimal_assignment = linear_sum_assignment(distances)

        movement = {robot: [] for robot in robots}

        for i in range(len(optimal_assignment[0])):
            assigned_agent = agents[optimal_assignment[1][i]]
            movement[robots[optimal_assignment[0][i]]].append(assigned_agent.loc)
            movement[robots[optimal_assignment[0][i]]].append(Point(assigned_agent.x,Consts.Y_SIZE))

        for robot in robots:
            robot.set_movement(movement[robot])

    def __str__(self):
        return 'RandomWalk10Planner'
