from agents.fixed_velocity_agent import FixedVelocityAgent
from planners.vertical_agent_greedy_planner import VerticalAgentGreedyPlanner
from robots.base_robot import BasicRobot
from simulator.environment import Environment
from utils.point import Point


if __name__ == '__main__':
    # agents = [UpwardsAgent(Point(x=random.uniform(0, Consts.SIZE_X), y=random.uniform(0, Consts.SIZE_Y)))
    #           for _ in range(Consts.AGENT_NUM)]
    # robots = [BaseRobot(Point(x=random.uniform(0, Consts.SIZE_X), y=random.uniform(0, Consts.SIZE_Y)))
    #           for _ in range(Consts.ROBOT_NUM)]

    agents = [FixedVelocityAgent(Point(5,5)), FixedVelocityAgent(Point(10,5))]
    robots = [BasicRobot(Point(5, 10)), BasicRobot(Point(10, 10))]

    env = Environment(agents=agents, robots=robots)
    planner = VerticalAgentGreedyPlanner(environment=env)

    is_finished = False
    while not is_finished:
        is_finished = env.advance()

    print(env.results())