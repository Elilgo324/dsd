import math

from planners.planner import Planner
from utils.algorithms import static_lack_moves, stochastic_lack_moves
from utils.functions import *
from world.stochastic_environment import StochasticEnvironment


class StochasticMonotoneLackPlanner(Planner):
    def __init__(self):
        self.alpha = math.pi / 8

    def plan(self, env: StochasticEnvironment):
        v, fv = env.agents[0].v, env.robots[0].fv
        sigma = env.agents[0].sigma
        d = env.robots[0].d

        vertical_fv = fv * math.cos(self.alpha)
        horizontal_fv = fv * math.sin(self.alpha)

        assert vertical_fv > v

        actual_agents = [StochasticAgent(loc=agent.loc, sigma=sigma, v=vertical_fv - v) for agent in env.agents]
        actual_robots = [BasicRobot(loc=Point(robot.x,env.border), fv=horizontal_fv, d=d) for robot in env.robots]

        stats = stochastic_lack_moves(actual_robots, actual_agents, h=env.border)
        disabled = stats['disabled']

        meeting_heights = {agent: meeting_height(BasicRobot(loc=Point(agent.x, 0), fv=vertical_fv), agent)
                           for agent in disabled}
        meeting_probs = {agent: round(
            integrate_gauss(mu=agent.x, sigma=sigma_t(sigma=sigma, t=meeting_heights[agent] - agent.y),
                            left=agent.x - d, right=agent.x + d), 3) for agent in disabled}

        expected_damage = sum([
            meeting_probs[agent] * (meeting_heights[agent] - agent.y)
            + (1 - meeting_probs[agent]) * (env.border - agent.y) for agent in disabled])
        completion_time = max(meeting_heights.values()) / vertical_fv

        mod_movement = stats['movement']
        movement = {robot: mod_movement[mod_robot] for robot, mod_robot in zip(env.robots, actual_robots)}

        return movement, \
               completion_time, \
               expected_damage, \
               stats['exp_disabled']

    def __str__(self):
        return f'StochasticMonotone{round((self.alpha * 360) / (2 * math.pi))}LackPlanner'
