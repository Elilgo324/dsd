from abc import ABC
from typing import Dict, List, Tuple

from world.robots.basic_robot import BasicRobot
from world.environment import Environment
from utils.point import Point


class Planner(ABC):
    def plan(self, environment: Environment):
        pass


