from abc import ABC
from typing import Dict, List

from agents.base_agent import BaseAgent
from robots.base_robot import BasicRobot
from environment import Environment
from utils.point import Point
from utils.consts import Consts


class Planner(ABC):
    def __init__(self, environment: Environment):
        self._environment = environment
        self._robots = environment.robots

        plan = self.plan()
        for robot in self._robots:
            robot.set_movement(plan[robot])

    def plan(self) -> Dict[BasicRobot, List[Point]]:
        pass


