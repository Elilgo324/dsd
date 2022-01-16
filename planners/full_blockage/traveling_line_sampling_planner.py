from typing import Dict, Tuple

from munkres import Munkres
# from scipy.optimize import linear_sum_assignment

import random
from planners.planner import Planner
from utils.functions import *


class TravelingLineSamplingPlanner(Planner):
    def __init__(self):
        self.lines_percentage = 10

    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, float, int]:
        robots = env.robots
        agents = env.agents
        movement = {robot: [] for robot in robots}

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
        optimal_assignment = Munkres().compute(distances)
        assigned_robots, assigned_agents = map(list, zip(*optimal_assignment))
        optimal_assignment = [assigned_robots, assigned_agents]

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

        # sample subset of H
        H = random.sample(H, int(len(H) / self.lines_percentage))

        h_makespan = {h: farthest_robot.loc.distance_to(Point(farthest_x, h)) / fv for h in H}
        h_trpv = {h: line_trpv(h, fv, agents, h_makespan[h]) for h in H}

        def damage_score(h):
            return h_trpv[h]['damage'] + (len(agents) * h_makespan[h])

        hs_damage_scores = {h: damage_score(h) for h in H}

        # assign robots on h opt
        h_opt = min(H, key=lambda h: hs_damage_scores[h])

        optimal_y = [h_opt] + h_trpv[h_opt]['ys']

        # add trp movement
        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[optimal_assignment[0][i]]
            for y in optimal_y:
                movement[assigned_robot].append(Point(optimal_x[assigned_robot], y))

        return movement, \
               h_trpv[h_opt]['t'] + h_makespan[h_opt], \
               h_trpv[h_opt]['t'] + h_makespan[h_opt], \
               hs_damage_scores[h_opt], \
               len(optimal_y)

    def __str__(self):
        return f'TravelingLineSampling{self.lines_percentage}Planner'
