from planners.planner import Planner
from utils.algorithms import iterative_assignment, stochastic_iterative_assignment
from utils.functions import *


class StochasticIterativePlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        stats = stochastic_iterative_assignment(env.robots, env.agents, env.border)
        return stats['movement'], \
               stats['completion_time'], \
               stats['expected_damage'], \
               stats['expected_num_disabled']

    def __str__(self):
        return 'StochasticIterativePlanner'
