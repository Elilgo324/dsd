import operator
from typing import Tuple, Dict

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class StaticLinePlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float]:
        robots = env.robots
        agents = env.agents
        movement = {robot: [] for robot in robots}

        b = env.border
        v = agents[0].v
        fv = robots[0].fv
        f = fv / v

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

        # map into pows of 2
        enumerate_object = enumerate([item for sublist in distances for item in sublist])
        sorted_pairs = sorted(enumerate_object, key=operator.itemgetter(1))
        sorted_indices = [index for index, element in sorted_pairs]

        pows = 2 ** -10
        for i in sorted_indices:
            row = i // len(robots)
            col = i % len(robots)
            distances[row][col] = pows
            pows *= 2

        optimal_assignment = linear_sum_assignment(distances)

        # optimal final x values
        optimal_x = {robots[optimal_assignment[0][i]]: locations[optimal_assignment[1][i]].x for i in range(len(
            optimal_assignment[0]))}

        # farthest robot
        chosen_distances = [distances[optimal_assignment[0][i]][optimal_assignment[1][i]] for i in
                            range(len(optimal_assignment[0]))]
        i_farthest_robot = max(optimal_assignment[0], key=lambda i: chosen_distances[i])
        farthest_robot = robots[i_farthest_robot]

        x_m0 = farthest_robot.x
        y_m0 = farthest_robot.y
        x_m = optimal_x[farthest_robot]

        def escaping_agents(h):
            i_escaping = []
            for i in range(len(agents)):
                y_a0 = agents[i].y
                if y_a0 > h - sqrt((x_m - x_m0) ** 2 + (h - y_m0) ** 2) / f:
                    i_escaping.append(i)
            return i_escaping

        def damage_score(h):
            damage = 0
            for i in range(len(agents)):
                y_a0 = agents[i].y
                damage += max(h - y_a0, 0)
            for i in escaping_agents(h):
                y_a0 = agents[i].y
                damage += min(b - h, b - y_a0)
            return damage

        # potential lines
        H = [meeting_height(farthest_robot, BaseAgent(Point(x_m, agent.y), agent.v)) for agent in agents]

        # assign on h opt
        h_opt = min(H, key=damage_score)

        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[optimal_assignment[0][i]]
            movement[assigned_robot].append(Point(optimal_x[assigned_robot], h_opt))

        makespan = farthest_robot.loc.distance_to(movement[farthest_robot][0]) / fv
        return movement, makespan

    def __str__(self):
        return 'StaticLinePlanner'
