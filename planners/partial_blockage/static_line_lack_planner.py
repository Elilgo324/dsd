from planners.planner import Planner
from utils.functions import *


class StaticLineLackPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, float, int]:
        robots = env.robots
        agents = env.agents

        b = env.border
        v = agents[0].v

        H = [meeting_height(robot, agent) for agent in agents for robot in robots]

        h_fm = {h: flow_moves(robots, agents, h) for h in H}
        h_non_escaping_agents = {h: h_fm[h]['disabled'] for h in H}
        h_movement = {h: h_fm[h]['movement'] for h in H}

        # calculate line score
        def damage_score(h):
            non_escaping = h_non_escaping_agents[h]
            damage = sum([b - agent.y for agent in agents])
            damage -= sum([b - h for _ in non_escaping])

            return damage

        h_damage_scores = {h: damage_score(h) for h in H}
        h_opt = min(H, key=lambda h: h_damage_scores[h])

        completion_time = 0
        if len(h_non_escaping_agents[h_opt]) > 0:
            completion_time = (h_opt - min([agent.y for agent in h_non_escaping_agents[h_opt]])) / v

        return h_movement[h_opt], \
               -1, \
               completion_time, \
               h_damage_scores[h_opt], \
               len(h_non_escaping_agents[h_opt])

    def __str__(self):
        return 'StaticLineLackPlanner'
