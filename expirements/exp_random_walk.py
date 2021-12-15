from random import seed

from agents.fixed_velocity_agent import FixedVelocityAgent
from planners.naive_planners.random_walk_10_planner import RandomWalk10Planner
from environment import Environment
from utils.functions import *


if __name__ == '__main__':
    seed(42)

    agents = [FixedVelocityAgent(sample_point()),
              FixedVelocityAgent(sample_point()),
              FixedVelocityAgent(sample_point()),
              FixedVelocityAgent(sample_point())]

    robots = [BasicRobot(sample_point()),
              BasicRobot(sample_point())]

    env = Environment(agents=agents, robots=robots)
    planner = RandomWalk10Planner(environment=env)

    is_finished = False
    while not is_finished:
        plot_environment(robots, agents)
        is_finished = env.advance()

    print(env.results())