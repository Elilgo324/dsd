from abc import ABC
from typing import Dict, List

from agents.base_agent import BaseAgent
from robots.basic_robot import BasicRobot
from environment import Environment
from utils.point import Point
from utils.consts import Consts


class Planner(ABC):
    def __init__(self, environment: Environment):
        self._environment = environment
        self._robots = environment.robots

    def plan(self) -> None:
        pass


