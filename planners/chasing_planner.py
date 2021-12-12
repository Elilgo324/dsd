
import random
from typing import Dict, List

from agents.base_agent import BaseAgent
from planners.planner import Planner
from robots.base_robot import BaseRobot
from simulator.environment import Environment
from utils.point import Point
from utils.consts import Consts


class VerticalAgentGreedyPlanner(Planner):
    def __init__(self, environment: Environment):
        super().__init__(environment)

    def plan(self) -> Dict[BaseRobot, List[Point]]:
        robots = self._environment.robots
        agents = self._environment.agents

        movement = {robot: [] for robot in robots}

        while agents:
            agent = agents.pop()
            robot = random.choice(robots)
            movement[robot].append(agent.loc)

        return movement

    def __str__(self):
        return 'VerticalAgentGreedyPlanner'
