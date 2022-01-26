from typing import Dict, Tuple

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class TravelingLinePlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents = env.agents
        movement = {robot: [] for robot in robots}

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
        optimal_assignment = linear_sum_assignment(map_into_2_pows(distances))

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
        makespan_per_h = {h: farthest_robot.loc.distance_to(Point(farthest_x, h)) / fv for h in H}
        trp_per_h = {h: line_trpv(h, fv, agents, makespan_per_h[h]) for h in H}

        def damage_score(h):
            return trp_per_h[h]['damage'] + (len(agents) * makespan_per_h[h])

        damage_score_per_h = {h: damage_score(h) for h in H}

        # assign robots on h opt
        h_opt = min(H, key=lambda h: damage_score_per_h[h])

        optimal_y = [h_opt] + trp_per_h[h_opt]['ys']

        # add trp movement
        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[optimal_assignment[0][i]]
            for y in optimal_y:
                movement[assigned_robot].append(Point(optimal_x[assigned_robot], y))

        return movement, \
               trp_per_h[h_opt]['t'] + makespan_per_h[h_opt], \
               damage_score_per_h[h_opt], \
               len(optimal_y)

    def __str__(self):
        return 'TravelingLinePlanner'
