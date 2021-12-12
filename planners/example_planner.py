
import random
from typing import Dict, List

from agents.base_agent import BaseAgent
from planners.planner import Planner
from robots.base_robot import BaseRobot
from simulator.environment import Environment
from utils.point import Point
from utils.consts import Consts


class ExamplePlanner(Planner):
    def __init__(self, environment: Environment):
        super().__init__(environment)

    def plan(self) -> Dict[BaseRobot, List[Point]]:
        robots = self._environment.robots
        agents = [agent.clone() for agent in self._environment.agents]

        movement = {robot: [] for robot in robots}

        movement[robots[0]] = [Point(robots[0].x + 5,robots[0].y),
                               Point(0,0),
                               Point(robots[-1].x - 5, robots[0].y)]
        movement[robots[-1]] = [Point(robots[-1].x - 5, robots[0].y),
                                Point(0,0),
                                Point(robots[0].x + 5,robots[0].y)]

        return movement

    def __str__(self):
        return 'ExamplePlanner'
