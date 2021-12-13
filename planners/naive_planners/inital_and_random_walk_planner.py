
import random
from typing import Dict, List

from agents.base_agent import BaseAgent
from planners.planner import Planner
from robots.base_robot import BasicRobot
from environment import Environment
from utils.point import Point
from utils.consts import Consts


class VerticalAgentGreedyPlanner(Planner):
    def __init__(self, environment: Environment):
        super().__init__(environment)

    def plan(self) -> Dict[BasicRobot, List[Point]]:
        robots = self._environment.robots
        agents = [agent.clone() for agent in self._environment.agents]

        movement = {robot: [] for robot in robots}

        while agents:
            agent = agents.pop()
            robot = random.choice(robots)
            movement[robot].append(agent.loc)
            movement[robot].append(Point(agent.x,Consts.Y_SIZE))

        return movement

    def check_meeting(self, robot: BasicRobot, agent: BaseAgent, limit=Consts.Y_SIZE):
        #TODO

        # calc here meeting loc and time
        meeting_loc = Point(0,0)
        meeting_time = 0

        return meeting_time, meeting_loc

    def __str__(self):
        return 'VerticalAgentGreedyPlanner'
