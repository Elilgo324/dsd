from random import random

from numpy.random import normal

from planners.planner import Planner
from planners.stochastic.partial_blockage.stochastic_static_lack_planner import StochasticStaticLackPlanner
from utils.functions import *
from world.agents.stochastic_agent import StochasticAgent
from world.stochastic_environment import StochasticEnvironment


class StochasticAdditiveLackPlanner(Planner):
    def __init__(self):
        self.num_waves = 2

    def plan(self, env: StochasticEnvironment):
        robots = env.robots
        agents = [a.clone() for a in env.agents]
        b = env.border
        v = agents[0].v
        fv = robots[0].fv
        r = robots[0].d
        sigma = agents[0].sigma

        movement = {robot: [] for robot in robots}
        completion_time = 0
        acc_damage = 0
        num_disabled = 0

        # divide into waves
        agents = sorted(agents, key= lambda a : a.y, reverse=True)
        wave_size = int(len(agents)/self.num_waves)
        waves = [agents[i:i + wave_size] for i in range(0, len(agents), wave_size)]

        # handle waves additively
        new_robots = robots
        cur_movement = {robot: [robot.loc] for robot in robots}
        for i_wave in range(len(waves)):
            wave = waves[i_wave]
            planner = StochasticStaticLackPlanner()
            new_robots = [BasicRobot(cur_movement[robot][-1], fv, r) for robot in new_robots]
            new_agents = [StochasticAgent(
                Point(normal(loc=agent.x, scale=sigma_t(sigma, int(v * completion_time))),
                      agent.y + v * completion_time), v, sigma) for agent in wave]
            new_env = StochasticEnvironment(robots=new_robots, agents=new_agents, top_border=env.top_border,
                                            left_border=env.left_border, right_border=env.right_border)
            cur_movement, cur_completion_time, cur_acc_damage, cur_num_disabled, _ = planner.plan(new_env)

            for i_robot in range(len(robots)):
                if len(cur_movement[new_robots[i_robot]]) == 0:
                    cur_movement[new_robots[i_robot]] = [new_robots[i_robot].loc]
                movement[robots[i_robot]] += cur_movement[new_robots[i_robot]]

            completion_time += cur_completion_time
            acc_damage += cur_acc_damage + wave_size * (len(waves) - i_wave - 1) * cur_completion_time
            num_disabled += cur_num_disabled

        return movement, completion_time, acc_damage, num_disabled

    def __str__(self):
        return f'StochasticAdditive{self.num_waves}LackPlanner'
