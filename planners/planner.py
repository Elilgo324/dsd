from abc import ABC
from typing import Dict, List, Tuple

from environment.robots.basic_robot import BasicRobot
from environment.environment import Environment
from utils.point import Point


class Planner(ABC):
    def plan(self, environment: Environment):
        pass


