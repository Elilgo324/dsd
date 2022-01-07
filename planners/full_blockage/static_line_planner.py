from typing import Tuple, Dict

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class StaticLinePlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents = env.agents
        movement = {robot: [] for robot in robots}

        b = env.border
        v = agents[0].v
        fv = robots[0].fv

        x_min_a = min(a.x for a in agents)
        x_max_a = max(a.x for a in agents)
        actual_range = (x_max_a - x_min_a) / len(robots)
        y_max_r = max(r.y for r in robots)

        # optimal assignment on bottom line
        locations = [Point(x_min_a + actual_range * (i + 0.5), y_max_r) for i in range(len(robots))]

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

        # calculate line score
        def damage_score(h):
            escaping, non_escaping = hs_escaping_agents[h]
            damage = 0

            for agent in escaping:
                damage += b - agent.y
            for agent in non_escaping:
                damage += h - agent.y
            return damage

        hs_damage_scores = {h: damage_score(h) for h in H}

        # assign robots on h opt
        h_opt = min(H, key=lambda h: hs_damage_scores[h])

        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[optimal_assignment[0][i]]
            movement[assigned_robot].append(Point(optimal_x[assigned_robot], h_opt))

        return movement, \
               h_makespan[h_opt], \
               hs_damage_scores[h_opt], \
               len(hs_escaping_agents[h_opt][1])

    def __str__(self):
        return 'StaticLinePlanner'
