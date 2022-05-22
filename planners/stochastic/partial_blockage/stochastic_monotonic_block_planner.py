from typing import Dict

from world.stochastic_environment import StochasticEnvironment
from planners.planner import Planner
from utils.flow_utils import *


class StochasticMonotonicBlockPlanner(Planner):
    def plan(self, env: StochasticEnvironment):
        robots = env.robots

        PA = env.PA
        UA = env.UA

        flow_data = stochastic_monotonic_block_moves(robots, UA, PA)
        utility = flow_data['utility']
        movement = flow_data['movement']
        timing = flow_data['timing']
        active_time = flow_data['active_time']
        expected_disabled = flow_data['expected_disabled']

        maximal_damage = sum([env.top_border - agent.y for agent in env.agents])

        return movement, \
               active_time, \
               maximal_damage - utility, \
               expected_disabled, \
               timing

    def __str__(self):
        return 'MonotonicBlockPlanner'
