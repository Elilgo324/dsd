import operator
from math import ceil
from math import sqrt

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class StaticLinePlanner(Planner):
    def plan(self, env: Environment) -> None:
        robots = env.robots
        agents = env.agents
        movement = {robot: [] for robot in robots}

        _, Y_SIZE = env.world_size
        d = robots[0].r
        v = agents[0].v
        fv = robots[0].fv
        f = fv / v

        x_min_a = min(a.x for a in agents)
        x_max_a = max(a.x for a in agents)
        y_max_r = max(r.y for r in robots)
        num_robots_on_block = ceil((x_max_a - x_min_a) / (2 * d))

        # optimal assignment on bottom line
        locations = [Point(x_min_a + d + 2 * d * i, y_max_r) for i in range(num_robots_on_block)]

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
                damage += min(Y_SIZE - h, Y_SIZE - y_a0)
            return damage

        H = []
        for a in agents:
            y_a0 = a.y
            H.append((f ** 2 * y_a0 + sqrt((f * y_a0 - f * y_m0) ** 2 + (x_m - x_m0) ** 2 * (f ** 2 - 1)))
                     / (f ** 2 - 1))

        h_opt = min(H, key=damage_score)

        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[optimal_assignment[0][i]]
            movement[assigned_robot].append(Point(optimal_x[assigned_robot], h_opt))

        for robot in robots:
            robot.set_movement(movement[robot])

    def __str__(self):
        return 'StaticLinePlanner'
