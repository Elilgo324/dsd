from environment.agents.stochastic_agent import StochasticAgent
from environment.stochastic_environment import StochasticEnvironment
from planners.planner import Planner
from planners.stochastic_movement.stochastic_static_lack_planner import StochasticStaticLackPlanner
from utils.functions import *


class StochasticAdditivePlanner(Planner):
    def __init__(self):
        self.wave_size = 10

    def plan(self, env: StochasticEnvironment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents = [a.clone() for a in env.agents]
        v = agents[0].v
        fv = robots[0].fv
        r = robots[0].d
        advance_distribution = agents[0].advance_distribution

        movement = {robot: [] for robot in robots}
        completion_time = 0
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
            new_agents = [StochasticAgent(Point(agent.x, agent.y + v * completion_time), v, advance_distribution)
                          for agent in wave]
            new_env = StochasticEnvironment(robots=new_robots, agents=new_agents, left_border=env.left_border,
                                            right_border=env.right_border, top_border=env.top_border)
            cur_movement, cur_completion_time, cur_damage, _ = planner.plan(new_env)

            for i_robot in range(len(robots)):
                if len(cur_movement[new_robots[i_robot]]) == 0:
                    cur_movement[new_robots[i_robot]] = [new_robots[i_robot].loc]
                movement[robots[i_robot]] += cur_movement[new_robots[i_robot]]

            completion_time += cur_completion_time
            acc_damage += cur_damage + self.wave_size * (len(waves) - i_wave - 1) * cur_completion_time

        return movement, 0, 0, 0

    def __str__(self):
        return 'StochasticAdditivePlanner'
