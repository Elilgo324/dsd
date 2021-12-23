import operator
from math import ceil
from math import sqrt
from typing import Dict, Tuple

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class TravelingLinePlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float]:
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

        pows = 2 ** -int(len(agents) / 2)
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

        def makespan(h):
            return sqrt((x_m - x_m0) ** 2 + (h - y_m0) ** 2) / fv

        def line_trpv(h):
            def time_to_meet(source, target):
                if source < target:
                    delta_y = target - source
                    time_to_meet = delta_y / (fv - v)
                else:
                    delta_y = source - target
                    time_to_meet = delta_y / (fv + v)
                return time_to_meet

            makespan_time = makespan(h)
            agents_ys = [a.y + v * makespan_time for a in agents]
            X = sorted([a for a in agents_ys if a > h])
            Y = sorted([a for a in agents_ys if a <= h], reverse=True)

            if len(X) == 0:
                time_to_meets = [time_to_meet(h, y) for y in agents_ys]
                return {'damage': sum(time_to_meets), 'ys': list(Y), 't': time_to_meets[-1]}

            XY = {x: {y: {'damage': 0, 'ys': [], 't': 0} for y in Y} for x in X}
            YX = {y: {x: {'damage': 0, 'ys': [], 't': 0} for x in X} for y in Y}

            # fill first row in both tables
            for i in range(len(Y)):
                x1 = X[0]
                yi = Y[i]
                time_h_yi = time_to_meet(h, yi)
                time_yi_x1 = time_to_meet(yi, x1)
                XY[x1][yi]['damage'] = sum([time_to_meet(h, y) for y in Y[:i + 1]]) \
                                       + time_h_yi + time_yi_x1
                XY[x1][yi]['ys'] = [yi + v * time_h_yi, x1 + v * (time_h_yi + time_yi_x1)]
                XY[x1][yi]['t'] = time_h_yi + time_yi_x1

            for i in range(len(X)):
                y1 = Y[0]
                xi = X[i]
                time_h_xi = time_to_meet(h, xi)
                time_xi_y1 = time_to_meet(xi, y1)
                YX[y1][xi]['damage'] = sum([time_to_meet(h, x) for x in X[:i]]) \
                                       + time_h_xi + time_xi_y1
                YX[y1][xi]['ys'] = [xi + v * time_h_xi, y1 + v * (time_h_xi + time_xi_y1)]
                YX[y1][xi]['t'] = time_h_xi + time_xi_y1

            # fill in diagonals
            for i in range(len(X) + len(Y) - 2):
                # top right XY
                idx_x = max(1, i - len(Y) + 2)
                idx_y = min(len(Y) - 1, i)

                # fill diagonal XY
                while idx_x < len(X) and idx_y >= 0:
                    num_living_agents = len(agents) - (idx_x + idx_y)

                    cur_x = X[idx_x]
                    prev_x = X[idx_x - 1]
                    cur_y = Y[idx_y]

                    time_reaching_from_up = time_to_meet(prev_x, cur_x)
                    time_reaching_from_down = time_to_meet(cur_y, cur_x)

                    damage_reaching_from_up = XY[prev_x][cur_y]['damage'] \
                                              + num_living_agents * time_reaching_from_up
                    damage_reaching_from_down = YX[cur_y][prev_x]['damage'] \
                                                + num_living_agents * time_reaching_from_down

                    if damage_reaching_from_up < damage_reaching_from_down:
                        XY[cur_x][cur_y]['damage'] = damage_reaching_from_up
                        XY[cur_x][cur_y]['t'] = XY[prev_x][cur_y]['t'] + time_reaching_from_up
                        path_reaching_from_up = XY[prev_x][cur_y]['ys']
                        XY[cur_x][cur_y]['ys'] = path_reaching_from_up + [cur_x + v * XY[cur_x][cur_y]['t']]
                    else:
                        XY[cur_x][cur_y]['damage'] = damage_reaching_from_down
                        XY[cur_x][cur_y]['t'] = YX[cur_y][prev_x]['t'] + damage_reaching_from_down
                        path_reaching_from_down = YX[cur_y][prev_x]['ys']
                        XY[cur_x][cur_y]['ys'] = path_reaching_from_down + [cur_x + v * XY[cur_x][cur_y]['t']]

                    idx_x += 1
                    idx_y -= 1

                # top right YX
                idx_y = max(1, i - len(X) + 2)
                idx_x = min(len(X) - 1, i)

                # fill diagonal YX
                while idx_y < len(Y) and idx_x >= 0:
                    num_living_agents = len(agents) - (idx_x + idx_y)

                    cur_y = Y[idx_y]
                    prev_y = Y[idx_y - 1]
                    cur_x = X[idx_x]

                    time_reaching_from_down = time_to_meet(prev_y, cur_y)
                    time_reaching_from_up = time_to_meet(cur_x, cur_y)

                    damage_reaching_from_down = XY[cur_x][prev_y]['damage'] \
                                                + num_living_agents * time_reaching_from_down
                    damage_reaching_from_up = YX[prev_y][cur_x]['damage'] \
                                              + num_living_agents * time_reaching_from_up

                    if damage_reaching_from_down < damage_reaching_from_up:
                        YX[cur_y][cur_x]['damage'] = damage_reaching_from_down
                        YX[cur_y][cur_x]['t'] = YX[prev_y][cur_x]['t'] + time_reaching_from_down
                        path_reaching_from_up = YX[prev_y][cur_x]['ys']
                        YX[cur_y][cur_x]['ys'] = path_reaching_from_up + [cur_y + v * YX[cur_y][cur_x]['t']]
                    else:
                        YX[cur_y][cur_x]['damage'] = damage_reaching_from_up
                        YX[cur_y][cur_x]['t'] = XY[cur_x][prev_y]['t'] + time_reaching_from_up
                        path_reaching_from_up = XY[cur_x][prev_y]['ys']
                        YX[cur_y][cur_x]['ys'] = path_reaching_from_up + [cur_y + v * YX[cur_y][cur_x]['t']]

                    idx_y += 1
                    idx_x -= 1

            damage_up = XY[X[-1]][Y[-1]]['damage']
            damage_down = YX[Y[-1]][X[-1]]['damage']

            if damage_up < damage_down:
                return XY[X[-1]][Y[-1]]
            return YX[Y[-1]][X[-1]]

        h_trpv = {h: line_trpv(h) for h in H}

        def damage_score(h):
            score = h_trpv[h]['damage']
            return score + (len(agents) * makespan(h)) / fv

        h_opt = min(H, key=damage_score)

        # refine optimal ys
        optimal_y = [h_opt] + h_trpv[h_opt]['ys'] + [Y_SIZE]

        refined_optimal_y = []
        for j in range(len(optimal_y)):
            if 0 < j < len(optimal_y) - 1 and optimal_y[j - 1] < optimal_y[j] < optimal_y[j + 1]:
                continue
            refined_optimal_y.append(optimal_y[j])

        # add trp movement
        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[optimal_assignment[0][i]]
            for y in refined_optimal_y:
                movement[assigned_robot].append(Point(optimal_x[assigned_robot], y))

        return movement, h_trpv[h_opt]['t']

    def __str__(self):
        return 'TravelingLinePlanner'
