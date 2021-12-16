from math import ceil
from random import seed

from agents.fixed_velocity_agent import FixedVelocityAgent
from planners.full_blockage.static_line_planner import StaticLinePlanner
from planners.naive_planners.offline_chasing_planner import OfflineChasingPlanner
from planners.naive_planners.online_chasing_planner import OnlineChasingPlanner
from planners.naive_planners.random_walk_10_planner import RandomWalk10Planner
from environment import Environment
from planners.planner import Planner
from utils.functions import *


if __name__ == '__main__':
    seed(42)

    agents = [FixedVelocityAgent(sample_point_between(0.1 * Consts.Y_SIZE, 0.5 * Consts.Y_SIZE)) for _ in range(
        Consts.AGENT_NUM)]

    y_min = min([a.y for a in agents])
    x_min = min([a.x for a in agents])
    x_max = max([a.x for a in agents])
    num_robots_for_full_blockage = ceil((x_max - x_min) / (2 * Consts.DISABLEMENT_RANGE))
    robots = [BasicRobot(sample_point_under(y_min)) for _ in range(num_robots_for_full_blockage)]

    env = Environment(agents=agents, robots=robots)
    # planner = RandomWalk10Planner(env)
    planner = OfflineChasingPlanner(env)
    # planner = OnlineChasingPlanner(env)
    # planner = StaticLinePlanner(environment=env)

    planner.plan()

    is_finished = False
    while not is_finished:
        # if env.step % 5 == 0:
        plot_environment(robots, agents, env)
        is_finished = env.advance()
    plot_environment(robots, agents, env)

    create_gif_from_plots()
    print('*** results ***')
    print(env.stats())