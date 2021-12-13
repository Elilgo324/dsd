from agents.fixed_velocity_agent import FixedVelocityAgent
from planners.naive_planners.inital_and_random_walk_planner import VerticalAgentGreedyPlanner
from robots.base_robot import BasicRobot
from environment import Environment
from utils.point import Point
import matplotlib.pyplot as plt


if __name__ == '__main__':
    agents = [FixedVelocityAgent(loc=Point(5, 5), v=0), FixedVelocityAgent(loc=Point(10, 5), v=0)]
    robots = [BasicRobot(loc=Point(0, 0)), BasicRobot(loc=Point(1, 1))]

    env = Environment(agents=agents, robots=robots)
    planner = VerticalAgentGreedyPlanner(environment=env)

    is_finished = False
    while not is_finished:
        plt.scatter([r.x for r in robots], [r.y for r in robots], c='blue')
        plt.scatter([a.x for a in agents], [a.y for a in agents], c='red')
        plt.show()
        is_finished = env.advance()

    print(env.results())