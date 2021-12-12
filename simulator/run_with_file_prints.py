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

    agents = [FixedVelocityAgent(loc=Point(5,5),v=0), FixedVelocityAgent(loc=Point(10,5),v=0)]
    robots = [BasicRobot(loc=Point(0, 0)), BasicRobot(loc=Point(1, 1))]

    env = Environment(agents=agents, robots=robots)
    planner = VerticalAgentGreedyPlanner(environment=env)

    is_finished = False
    while not is_finished:
        is_finished = env.advance()

    print(env.results())