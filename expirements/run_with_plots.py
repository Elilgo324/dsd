from agents.fixed_velocity_agent import FixedVelocityAgent
from planners.naive_planners.inital_and_random_walk_planner import VerticalAgentGreedyPlanner
from robots.base_robot import BasicRobot
from environment import Environment
from utils.point import Point
import matplotlib.pyplot as plt

if __name__ == '__main__':
    agents = [FixedVelocityAgent(Point(5,5)), FixedVelocityAgent(Point(10,5))]
    robots = [BasicRobot(Point(5, 10)), BasicRobot(Point(10, 10))]

    env = Environment(agents=agents, robots=robots)
    planner = VerticalAgentGreedyPlanner(environment=env)



    is_finished = False
    while not is_finished:
        is_finished = env.advance()

    print(env.results())