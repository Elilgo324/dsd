from environment.stochastic_environment import StochasticEnvironment
from planners.planner import Planner
from utils.functions import *


class StochasticStaticLackPlanner(Planner):
    def plan(self, env: StochasticEnvironment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots

        PA = env.generate_PA()
        UA = env.generate_UA(PA)
        U = env.generate_U(UA)

        T,num_rows,num_cols = U.shape

        flow_per_h = {h: stochastic_lack_moves(robots, h, U) for h in range(num_rows)}
        utility_per_h = {h: flow_per_h[h]['utility'] for h in range(num_rows)}
        movement_per_h = {h: flow_per_h[h]['movement'] for h in range(num_rows)}

        h_opt = min(list(range(
            num_rows)), key=lambda h: utility_per_h[h])

        completion_time = 0

        return movement_per_h[h_opt], \
               completion_time, \
               utility_per_h[h_opt], \
               0

    def __str__(self):
        return 'StochasticStaticLackPlanner'
