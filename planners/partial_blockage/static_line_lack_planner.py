from typing import Dict

from planners.planner import Planner
from utils.functions import *


class StaticLineLackPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, float, int]:
        robots = env.robots
        agents = env.agents
        movement = {robot: [] for robot in robots}

        b = env.border
        v = agents[0].v
        fv = robots[0].fv

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






        x_min_a = min(a.x for a in agents)
        x_max_a = max(a.x for a in agents)
        actual_range = (x_max_a - x_min_a) / len(robots)
        y_min_a = min(a.y for a in agents)

        # optimal assignment on bottom line
        locations = [Point(x_min_a + actual_range * (i + 0.5), y_min_a) for i in range(len(robots))]

        # hungarian algorithm minimizing makespan
        distances = [[] for _ in range(len(robots))]
        for i in range(len(robots)):
            distances[i] = [robots[i].loc.distance_to(locations[j]) for j in range(len(locations))]

        modified_distances = map_into_2_pows(distances)
        optimal_assignment = linear_sum_assignment(modified_distances)

        # optimal final x values
        optimal_x = {robots[optimal_assignment[0][i]]: locations[optimal_assignment[1][i]].x for i in range(len(
            optimal_assignment[0]))}

        # farthest robot
        chosen_distances = [distances[optimal_assignment[0][i]][optimal_assignment[1][i]] for i in
                            range(len(optimal_assignment[0]))]
        farthest_robot = robots[max(optimal_assignment[0], key=lambda i: chosen_distances[i])]
        farthest_x = optimal_x[farthest_robot]

        # potential lines
        H = [meeting_height(farthest_robot, BaseAgent(Point(farthest_x, agent.y), agent.v)) for agent in agents]
        h_makespan = {h: farthest_robot.loc.distance_to(Point(farthest_x, h)) / fv for h in H}

        # group agents into escaping and non escaping
        def escaping_agents(h) -> tuple[list[BaseAgent], list[BaseAgent]]:
            escaping, non_escaping = [], []
            for agent in agents:
                if agent.y + h_makespan[h] * v > h:
                    escaping.append(agent)
                else:
                    non_escaping.append(agent)
            return escaping, non_escaping

        hs_escaping_agents = {h: escaping_agents(h) for h in H}

        # assign robots on h opt


        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[optimal_assignment[0][i]]
            movement[assigned_robot].append(Point(optimal_x[assigned_robot], h_opt))



    def __str__(self):
        return 'StaticLineLackPlanner'
