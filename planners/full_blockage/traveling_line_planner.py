import math
import operator
from math import ceil
from math import sqrt

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class TravelingLinePlanner(Planner):
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

        H = []
        for a in agents:
            y_a0 = a.y
            H.append((f ** 2 * y_a0 + sqrt((f * y_a0 - f * y_m0) ** 2 + (x_m - x_m0) ** 2 * (f ** 2 - 1)))
                     / (f ** 2 - 1))

        def line_trpv(h):
            def time_to_meet(source, target):
                if source < target:
                    delta_y = target - source
                    time_to_meet = delta_y / (fv + v)
                else:
                    delta_y = source - target
                    time_to_meet = delta_y / (fv - v)
                return time_to_meet

            agents_ys = [a.y for a in agents]
            A_up = [h] + sorted([a for a in agents_ys if a >= h])
            A_down = [h] + sorted([a for a in agents_ys if a < h], reverse=True)

            T = {a1: {a2: {'damage': 0, 'ys': []} for a2 in A_up + A_down} for a1 in A_up + A_down}
            T[h][h] = {'damage': 0, 'ys': [h]}

            for i in range(len(A_up)):
                T[h][A_up[i]] = {'damage': math.inf, 'ys': []}
                T[A_up[i]][h] = {'damage': time_to_meet(h, A_up[i]) * len(agents), 'ys': [h, A_up[i]]}

            for i in range(len(A_down)):
                T[h][A_down[i]] = {'damage': math.inf, 'ys': []}
                T[A_down[i]][h] = {'damage': time_to_meet(h, A_down[i]) * len(agents), 'ys': [h, A_down[i]]}

            for i in range(1, max(len(A_up), len(A_down))):
                for j in range(0, max(len(A_up), len(A_down))):
                    num_living_agents = len(agents) + 1 - (i + j)

                    if i < len(A_up) and j < len(A_down):
                        time_reaching_from_up = time_to_meet(A_up[i - 1], A_up[i])
                        time_reaching_from_down = time_to_meet(A_down[j], A_up[i])

                        damage_reaching_from_up = T[A_up[i - 1]][A_down[j]]['damage'] \
                                                  + num_living_agents * time_reaching_from_up
                        damage_reaching_from_down = T[A_down[j]][A_up[i - 1]]['damage'] \
                                                    + num_living_agents * time_reaching_from_down

                        if damage_reaching_from_up < damage_reaching_from_down:
                            T[A_up[i]][A_down[j]]['damage'] = damage_reaching_from_up
                            path_reaching_from_up = T[A_up[i - 1]][A_down[j]]['ys']
                            T[A_up[i]][A_down[j]]['path'] = path_reaching_from_up + [A_up[i]]
                        else:
                            T[A_up[i]][A_down[j]]['damage'] = damage_reaching_from_down
                            path_reaching_from_down = T[A_down[j]][A_up[i - 1]]['ys']
                            T[A_up[i]][A_down[j]]['path'] = path_reaching_from_down + [A_up[i]]

                    if j < len(A_up) and i < len(A_down):
                        time_reaching_from_up = time_to_meet(A_up[j], A_down[i])
                        time_reaching_from_down = time_to_meet(A_down[i - 1], A_down[i])

                        damage_reaching_from_up = T[A_up[j]][A_down[i - 1]]['damage'] \
                                                  + num_living_agents * time_reaching_from_up
                        damage_reaching_from_down = T[A_down[i - 1]][A_up[j]]['damage'] \
                                                    + num_living_agents * time_reaching_from_down

                        if damage_reaching_from_up < damage_reaching_from_down:
                            T[A_down[i]][A_up[j]]['damage'] = damage_reaching_from_up
                            path_reaching_from_up = T[A_up[j]][A_down[i - 1]]['ys']
                            T[A_down[i]][A_up[j]]['path'] = path_reaching_from_up + [A_down[i]]
                        else:
                            T[A_down[i]][A_up[j]]['damage'] = damage_reaching_from_down
                            path_reaching_from_down = T[A_down[i - 1]][A_up[j]]['ys']
                            T[A_down[i]][A_up[j]]['path'] = path_reaching_from_down + [A_down[i]]

            damage_up, movement_up = T[A_up[-1]][A_down[-1]]['damage'], T[A_up[-1]][A_down[-1]]['ys']
            damage_down, movement_down = T[A_down[-1]][A_up[-1]]['damage'], T[A_down[-1]][A_up[-1]]['ys']
            if damage_up < damage_down:
                return damage_up, movement_up
            return damage_down, movement_down

        h_scores_movements = {h: line_trpv(h) for h in H}

        def t_max(h):
            return sqrt((x_m - x_m0) ** 2 + (h - y_m0) ** 2) / fv

        def damage_score(h):
            score, _ = line_trpv(h)
            return score + (len(agents) * t_max(h)) / fv

        h_opt = min(H, key= lambda h:h_scores_movements[h][0])

        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[optimal_assignment[0][i]]
            movement[assigned_robot].append(Point(optimal_x[assigned_robot], h_opt))

        trp_ys = h_scores_movements[h_opt][1]
        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[optimal_assignment[0][i]]
            for y in trp_ys:
                movement[assigned_robot].append(Point(optimal_x[assigned_robot], y))

        for robot in robots:
            robot.set_movement(movement[robot])

    def __str__(self):
        return 'StaticLinePlanner'
