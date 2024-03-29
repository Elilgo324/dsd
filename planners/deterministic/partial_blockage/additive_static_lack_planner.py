from world.agents.stochastic_agent import StochasticAgent
from world.stochastic_environment import StochasticEnvironment
from planners.deterministic.partial_blockage.static_line_lack_planner import StaticLineLackPlanner
from planners.planner import Planner
from utils.functions import *


class AdditiveStaticLackPlanner(Planner):
    def __init__(self):
        self.wave_size = 100

    def plan(self, env: StochasticEnvironment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents = [a.clone() for a in env.agents]
        b = env.border
        bl = env.left_border
        br = env.right_border
        v = agents[0].v
        fv = robots[0].fv
        r = robots[0].d
        advance_distribution = agents[0].advance_distribution

        movement = {robot: [] for robot in robots}
        completion_time = 0
        acc_damage = 0
        num_disabled = 0

        # divide into waves
        agents = sorted(agents, key= lambda a : a.y, reverse=True)
        wave_size = int(len(agents) / self.wave_size)
        waves = [agents[i:i + wave_size] for i in range(0, len(agents), wave_size)]

        # handle waves additively
        new_robots = robots
        cur_movement = {robot: [robot.loc] for robot in robots}
        for i_wave in range(len(waves)):
            wave = waves[i_wave]
            planner = StaticLineLackPlanner()
            new_robots = [BasicRobot(cur_movement[robot][-1], fv, r) for robot in new_robots]
            new_agents = [StochasticAgent(Point(agent.x, agent.y + v * completion_time), v, advance_distribution) for agent in wave]
            new_env = StochasticEnvironment(robots=new_robots, agents=new_agents, top_border=b, right_border=br, left_border=bl)
            cur_movement, cur_completion_time, cur_acc_damage, cur_num_disabled = planner.plan(new_env)

            for i_robot in range(len(robots)):
                if len(cur_movement[new_robots[i_robot]]) == 0:
                    cur_movement[new_robots[i_robot]] = [new_robots[i_robot].loc]
                movement[robots[i_robot]] += cur_movement[new_robots[i_robot]]

            completion_time += cur_completion_time
            acc_damage += cur_acc_damage + wave_size * (len(waves) - i_wave - 1) * cur_completion_time
            num_disabled += cur_num_disabled

        return movement, completion_time, acc_damage, num_disabled

    def __str__(self):
        return f'Additive{self.wave_size}StaticLackPlanner'
