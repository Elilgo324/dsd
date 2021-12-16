from math import ceil
from random import seed

from agents.fixed_velocity_agent import FixedVelocityAgent
from config import *
from planners.full_blockage.static_line_planner import StaticLinePlanner
from planners.naive_planners.offline_chasing_planner import OfflineChasingPlanner
from planners.naive_planners.online_chasing_planner import OnlineChasingPlanner
from planners.naive_planners.random_walk_10_planner import RandomWalk10Planner
from planners.planner import Planner
from utils.functions import *


def run(planner: Planner):
    seed(SEED)

    agents = [FixedVelocityAgent(sample_point(0, X_SIZE, 0.1 * Y_SIZE, 0.5 * Y_SIZE), AGENT_SPEED)
              for _ in range(AGENT_NUM)]

    y_min = min([a.y for a in agents])
    x_min = min([a.x for a in agents])
    x_max = max([a.x for a in agents])
    num_robots_for_full_blockage = ceil((x_max - x_min) / (2 * DISABLEMENT_RANGE))
    robots = [BasicRobot(sample_point(0, X_SIZE, 0, y_min), ROBOT_SPEED, DISABLEMENT_RANGE)
              for _ in range(num_robots_for_full_blockage)]

    env = Environment(agents=agents, robots=robots, world_size=SIZE)

    planner.plan(env)

    is_finished = False
    while not is_finished:
        # if env.step % 5 == 0:
        plot_environment(robots, agents, env)
        is_finished = env.advance()
    plot_environment(robots, agents, env)
    create_gif_from_plots(prefix=str(planner))

    print(f'*** results of {str(planner)} ***')
    print(env.stats())


if __name__ == '__main__':
    planners = [RandomWalk10Planner(), OfflineChasingPlanner(), OnlineChasingPlanner(), StaticLinePlanner()]
    for planner in planners:
        run(planner)
