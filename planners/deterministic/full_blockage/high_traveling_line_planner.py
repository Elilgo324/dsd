from world.agents.deterministic_agent import DeterministicAgent
from planners.deterministic.full_blockage.traveling_line_planner import TravelingLinePlanner
from planners.planner import Planner
from utils.functions import *


class HighTravelingLinePlanner(Planner):
    def __init__(self):
        self.max_agents = 2

    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents = env.agents

        r = robots[0].d
        fv = robots[0].fv
        v = agents[0].v
        b = env.border

        x_min_a = min(a.x for a in agents)
        x_max_a = max(a.x for a in agents)

        cur_max_agents = min(len(agents), self.max_agents)

        agents = sorted(agents, key=lambda a: a.y, reverse=True)
        trp_agents = agents[:cur_max_agents]
        scanner_agents = agents[cur_max_agents:]

        # trp part
        trp_agents = [DeterministicAgent(Point(x_min_a, trp_agents[0].y), v=v),
                      DeterministicAgent(Point(x_max_a, trp_agents[1].y), v=v)] + trp_agents[2:]
        trp_env = Environment(robots=robots, agents=trp_agents, border=b)
        movement, completion_time, damage, num_disabled = TravelingLinePlanner().plan(trp_env)

        # scanner part
        if len(scanner_agents) == 0:
            return movement, completion_time, damage, num_disabled

        damage += completion_time * len(scanner_agents) * v

        prev_height = movement[robots[0]][-1].y
        for i_agent in range(len(scanner_agents)):
            agent = scanner_agents[i_agent]
            meeting_h = meeting_height(BasicRobot(loc=Point(0, prev_height), r=r, fv=fv),
                                       BaseAgent(loc=Point(0, agent.y + completion_time * v), v=v))
            cur_completion_time = (prev_height - meeting_h) / fv
            prev_height = meeting_h
            for robot in robots:
                movement[robot].append(Point(movement[robot][-1].x, meeting_h))

            completion_time += cur_completion_time
            damage += (len(scanner_agents) - i_agent) * cur_completion_time * v
            num_disabled += 1

        return movement, completion_time, damage, num_disabled

    def __str__(self):
        return f'High{self.max_agents}TravelingLinePlanner'
