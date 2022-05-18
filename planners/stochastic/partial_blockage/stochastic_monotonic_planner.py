from typing import Dict

from world.stochastic_environment import StochasticEnvironment
from planners.planner import Planner
from utils.flow_utils import *


class StochasticMonotonicPlanner(Planner):
    def plan(self, env: StochasticEnvironment):
        robots = env.robots

        PA = env.PA
        UA = env.UA

        T, num_rows, num_cols = UA.shape

        flow_per_h = {h: stochastic_monotonic_moves(robots, UA, PA) for h in range(num_rows)}
        utility_per_h = {h: flow_per_h[h]['utility'] for h in range(num_rows)}
        movement_per_h = {h: flow_per_h[h]['movement'] for h in range(num_rows)}
        timing_per_h = {h: flow_per_h[h]['timing'] for h in range(num_rows)}
        active_time_per_h = {h: flow_per_h[h]['active_time'] for h in range(num_rows)}
        expected_disabled_per_h = {h: flow_per_h[h]['expected_disabled'] for h in range(num_rows)}

        maximal_damage = sum([env.top_border - agent.y for agent in env.agents])

        h_opt = max(list(range(num_rows)), key=lambda h: utility_per_h[h])

        return movement_per_h[h_opt], \
               active_time_per_h[h_opt], \
               maximal_damage - utility_per_h[h_opt], \
               expected_disabled_per_h[h_opt], \
               timing_per_h[h_opt]

    def __str__(self):
        return 'StochasticStaticLackPlanner'
