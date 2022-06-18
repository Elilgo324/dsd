from planners.planner import Planner
from utils.algorithms import static_lack_moves, stochastic_lack_moves
from utils.functions import *


class StochasticMonotoneLackPlanner(Planner):
    def plan(self, env: Environment):
        v, fv = env.agents[0].v, env.robots[0].fv
        vertical_fv = 0.75 * fv
        horizontal_fv = 0.25 * fv

        actual_agents = [agent.clone() for agent in env.agents]
        for aa in actual_agents:
            aa.v = fv - v

        actual_robots = [robot.clone() for robot in env.robots]
        for ar in actual_robots:
            ar.fv = horizontal_fv
            ar.loc = Point(ar.x, env.border)

        stats = stochastic_lack_moves(actual_robots, actual_agents, h=env.border)
        disabled = stats['disabled']

        meeting_heights = {agent: meeting_height(BasicRobot(loc=Point(agent.x, 0), fv=vertical_fv), agent) for agent in
                           disabled}

        expected_damage = sum([meeting_heights[agent] - agent.y for agent in disabled])
        completion_time = max(meeting_heights.values())

        return stats['movement'], \
               completion_time, \
               expected_damage, \
               stats['exp_disabled']

    def __str__(self):
        return 'StochasticMonotoneLackPlanner'
