from random import seed

from agents.fixed_velocity_agent import FixedVelocityAgent
from planners.full_blockage.static_line_planner import StaticLinePlanner
from planners.naive_planners.random_walk_10_planner import RandomWalk10Planner
from environment import Environment
from utils.functions import *


if __name__ == '__main__':
    seed(42)

    agents = [FixedVelocityAgent(sample_point()) for _ in range(4)]

    y_min = min([a.y for a in agents])
    x_min = min([a.x for a in agents])
    x_max = max([a.x for a in agents])
    num_robots_for_full_blockage = (x_max - x_min) / (2 * Consts.DISABLEMENT_RANGE)
    robots = [BasicRobot(sample_point_under(y_min)) for _ in range(int(num_robots_for_full_blockage))]

    env = Environment(agents=agents, robots=robots)
    planner = StaticLinePlanner(environment=env)
    # planner = RandomWalk10Planner(env)

    is_finished = False
    i = 0
    while not is_finished:
        if i % 10 == 0:
            plot_environment(robots, agents)
        i += 1
        is_finished = env.advance()


    print(env.results())