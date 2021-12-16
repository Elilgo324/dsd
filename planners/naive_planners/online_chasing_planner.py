import random
from typing import Dict, List

from scipy.optimize import linear_sum_assignment

from agents.base_agent import BaseAgent
from planners.planner import Planner
from robots.basic_robot import BasicRobot
from environment import Environment
from utils.functions import *


class OnlineChasingPlanner(Planner):
    def plan(self, env: Environment) -> None:
        robots = env.robots
        agents = env.agents

        distances = [[] for _ in range(len(robots))]
        for i in range(len(robots)):
            distances[i] = [robots[i].loc.distance_to(agents[j].loc) for j in range(len(agents))]

        optimal_assignment = linear_sum_assignment(distances)

        movement = {robot: [] for robot in robots}

        for i in range(len(optimal_assignment[0])):
            movement[robots[optimal_assignment[0][i]]].append(agents[optimal_assignment[1][i]])

        for robot in robots:
            robot.set_movement(movement[robot])

    def __str__(self):
        return 'OnlineChasingPlanner'
