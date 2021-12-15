from agents.fixed_velocity_agent import FixedVelocityAgent
from planners.naive_planners.random_walk_10_planner import VerticalAgentGreedyPlanner
from robots.basic_robot import BasicRobot
from environment import Environment
from utils.point import Point


if __name__ == '__main__':
    agents = [FixedVelocityAgent(Point(5,5)), FixedVelocityAgent(Point(10,5))]
    robots = [BasicRobot(Point(5, 3)), BasicRobot(Point(9, 2))]

    env = Environment(agents=agents, robots=robots)
    planner = VerticalAgentGreedyPlanner(environment=env)

    is_finished = False
    while not is_finished:
        is_finished = env.advance()

    print(env.results())