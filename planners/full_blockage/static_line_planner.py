import random
from typing import Dict, List

from agents.base_agent import BaseAgent
from planners.planner import Planner
from robots.basic_robot import BasicRobot
from environment import Environment
from utils.functions import *
import networkx as nx
from scipy.optimize import linear_sum_assignment


class StaticLinePlanner(Planner):
    def __init__(self, environment: Environment):
        super().__init__(environment)

    def plan(self) -> Dict[BasicRobot, List[Point]]:
        robots = self._environment.robots
        agents = self._environment.agents

        movement = {robot: [] for robot in robots}

        x_min = min(a.x for a in agents)
        x_max = max(a.x for a in agents)
        y_min = min(a.y for a in agents)
        y_max = max(a.y for a in agents)

        num_robots_on_block = (x_max - x_min) / (2 * Consts.DISABLEMENT_RANGE)
        locations = [Point(x_min - Consts.DISABLEMENT_RANGE + 2*Consts.DISABLEMENT_RANGE * i,
                           y_max) for i in range(int(num_robots_on_block))]
        distances = [[] for _ in range(len(robots))]
        for i in range(len(robots)):
            distances[i] = [robots[i].loc.distance_to(locations[j]) for j in range(len(locations))]

        optimal_assignment = linear_sum_assignment(distances)
        print(optimal_assignment)


        return movement

    def __str__(self):
        return 'RandomWalk10Planner'
