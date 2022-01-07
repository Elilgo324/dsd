from typing import Dict, Tuple

from planners.planner import Planner
from utils.functions import *


class IterativeAssignmentPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents_copy = [a.clone() for a in env.agents]
        return iterative_assignment(robots, agents_copy).values()

    def __str__(self):
        return 'IterativeAssignmentPlanner'
