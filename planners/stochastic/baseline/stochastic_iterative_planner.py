from planners.planner import Planner
from utils.algorithms import iterative_assignment, stochastic_iterative_assignment
from utils.functions import *


class StochasticIterativePlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents_copy = [a.clone() for a in env.agents]
        stats = stochastic_iterative_assignment(robots, agents_copy, env.border)
        return stats['movement'], \
               stats['completion_time'], \
               stats['damage'], \
               stats['num_disabled']

    def __str__(self):
        return 'StochasticIterativePlanner'
