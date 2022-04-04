from planners.planner import Planner
from utils.functions import *


class LowStaticLineLacklPlanner(Planner):
    def __init__(self):
        self.max_agents = 100

    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents = env.agents
        b = env.border
        v = agents[0].v

        cur_max_agents = min(len(agents), self.max_agents)

        agents = sorted(agents, key=lambda a: a.y)
        active_agents = agents[:cur_max_agents]
        giveup_agents = agents[cur_max_agents:]
        giveup_damage = sum([env.border - a.y for a in giveup_agents])

        agents = active_agents

        H = [meeting_height(robot, agent) for agent in agents for robot in robots]

        flow_per_h = {h: static_lack_moves(robots, agents, h) for h in H}
        disabled_per_h = {h: flow_per_h[h]['disabled'] for h in H}
        movement_per_h = {h: flow_per_h[h]['movement'] for h in H}

        # calculate line score
        def damage_score(h):
            non_escaping = disabled_per_h[h]
            damage = sum([b - agent.y for agent in agents])
            damage -= sum([b - h for _ in non_escaping])
            return damage

        damage_score_per_h = {h: damage_score(h) for h in H}
        h_opt = min(H, key=lambda h: damage_score_per_h[h])

        completion_time = 0
        if len(disabled_per_h[h_opt]) > 0:
            completion_time = (h_opt - min([agent.y for agent in disabled_per_h[h_opt]])) / v

        return movement_per_h[h_opt], \
               completion_time, \
               damage_score_per_h[h_opt] + giveup_damage, \
               len(disabled_per_h[h_opt])

    def __str__(self):
        return f'Low{self.max_agents}StaticLineLackPlanner'
