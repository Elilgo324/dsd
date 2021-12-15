import random
from typing import Dict, List

from agents.base_agent import BaseAgent
from planners.planner import Planner
from robots.basic_robot import BasicRobot
from environment import Environment
from utils.functions import *


class RandomWalk10Planner(Planner):
    def __init__(self, environment: Environment):
        super().__init__(environment)

    def plan(self) -> Dict[BasicRobot, List[Point]]:
        robots = self._environment.robots

        movement = {robot: [] for robot in robots}

        for _ in range(10):
            for robot in robots:
                movement[robot].append(sample_point())

        return movement

    def __str__(self):
        return 'RandomWalk10Planner'
