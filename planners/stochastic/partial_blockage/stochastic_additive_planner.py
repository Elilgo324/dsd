from random import choices

from world.agents.stochastic_agent import StochasticAgent
from world.stochastic_environment import StochasticEnvironment
from planners.planner import Planner
from planners.stochastic.partial_blockage.stochastic_static_lack_planner import StochasticStaticLackPlanner
from utils.functions import *


class StochasticAdditivePlanner(Planner):
    def __init__(self):
        self.wave_size = 3

    def plan(self, env: StochasticEnvironment):
        robots = env.robots
        agents = [a.clone() for a in env.agents]
        v = agents[0].v
        fv = robots[0].fv
        r = robots[0].d
        advance_distribution = agents[0].advance_distribution
        bl = env.left_border
        br = env.right_border

        movement = {robot: [] for robot in robots}
        timing = {robot: [] for robot in robots}
        active_time = 0
        acc_damage = 0
        num_disabled = 0

        # divide into waves
        agents = sorted(agents, key=lambda a: a.y, reverse=True)
        waves = [agents[i:i + self.wave_size] for i in range(0, len(agents), self.wave_size)]

        # handle waves additively
        new_robots = robots
        cur_movement = {robot: [robot.loc] for robot in robots}
        for i_wave, wave in enumerate(waves):
            planner = StochasticStaticLackPlanner()
            new_robots = [BasicRobot(cur_movement[robot][-1], fv, r) for robot in new_robots]
            noise = sum([choices([-1, 0, 1], weights=advance_distribution, k=active_time)[0] for _ in range(active_time)])
            new_agents = [StochasticAgent(Point(agent.x + noise, agent.y + v * active_time),
                                          v, advance_distribution, bl, br) for agent in wave
                          if agent.y + v * active_time < env.top_border]
            new_env = StochasticEnvironment(robots=new_robots, agents=new_agents, left_border=env.left_border,
                                            right_border=env.right_border, top_border=env.top_border)
            cur_movement, cur_active_time, cur_damage, _, cur_timing = planner.plan(new_env)

            for i_robot, robot in enumerate(new_robots):
                movement[robots[i_robot]] += cur_movement[robot]
                timing[robots[i_robot]] += cur_timing[robot]

            active_time += cur_active_time
            acc_damage += cur_damage + self.wave_size * (len(waves) - i_wave - 1) * cur_active_time

        return movement, 0, 0, 0, timing

    def __str__(self):
        return 'StochasticAdditivePlanner'
