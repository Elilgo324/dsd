from planners.planner import Planner
from utils.algorithms import static_lack_moves
from utils.functions import *


class StaticLineLackPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents = env.agents
        b = env.border
        v = agents[0].v

        meeting_heights = {robot: {agent: meeting_height(robot, agent) for agent in agents} for robot in robots}
        H = [meeting_heights[robot][agent] for agent in agents for robot in robots if meeting_heights[robot][agent] < b]
        if len(H) == 0:
            return {robot: [robot.loc] for robot in robots}, 0, sum([b - agent.y for agent in agents]), 0

        flow_per_h = {h: static_lack_moves(robots, agents, h) for h in H}
        disabled_per_h = {h: flow_per_h[h]['disabled'] for h in H}
        movement_per_h = {h: flow_per_h[h]['movement'] for h in H}

        # calculate line score
        def damage_score(h):
            return sum([b - agent.y for agent in agents]) - len(disabled_per_h[h]) * (b - h)

        damage_score_per_h = {h: damage_score(h) for h in H}
        h_opt = min(H, key=lambda h: damage_score_per_h[h])

        completion_time = 0
        if len(disabled_per_h[h_opt]) > 0:
            completion_time = (h_opt - min([agent.y for agent in disabled_per_h[h_opt]])) / v

        return movement_per_h[h_opt], \
               completion_time, \
               damage_score_per_h[h_opt], \
               len(disabled_per_h[h_opt])

    def __str__(self):
        return 'StaticLineLackPlanner'
