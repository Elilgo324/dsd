import operator
from math import ceil

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *
from math import sqrt


def makespan(h, x_m, x_m0, y_m0):
    return sqrt((x_m - x_m0)**2 + (h - y_m0)**2) / Consts.ROBOT_DEF_SPEED


def escaping_agents(h):
    pass


def damage_score(h):
    pass


class StaticLinePlanner(Planner):
    def __init__(self, environment: Environment):
        super().__init__(environment)

    def plan(self) -> None:
        robots = self._environment.robots
        agents = self._environment.agents
        movement = {robot: [] for robot in robots}

        x_min_a = min(a.x for a in agents)
        x_max_a = max(a.x for a in agents)

        y_max_r = max(r.y for r in robots)

        num_robots_on_block = ceil((x_max_a - x_min_a) / (2 * Consts.DISABLEMENT_RANGE))

        # optimal assignment on bottom line
        locations = [Point(x_min_a + Consts.DISABLEMENT_RANGE + 2 * Consts.DISABLEMENT_RANGE * i,
                           y_max_r) for i in range(num_robots_on_block)]

        distances = [[] for _ in range(len(robots))]
        for i in range(len(robots)):
            distances[i] = [robots[i].loc.distance_to(locations[j]) for j in range(len(locations))]

        enumerate_object = enumerate([item for sublist in distances for item in sublist])
        sorted_pairs = sorted(enumerate_object, key=operator.itemgetter(1))
        sorted_indices = [index for index, element in sorted_pairs]

        pows = 1
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
        chosen_distances = [distances[optimal_assignment[0][i]][optimal_assignment[1][i]] for i in range(len(optimal_assignment[0]))]
        i_farthest_robot = max(optimal_assignment[0], key=lambda i: chosen_distances[i])
        farthest_robot = robots[i_farthest_robot]
        x_m0 = farthest_robot.x
        y_m0 = farthest_robot.y
        x_m = optimal_x[farthest_robot]


        for i in range(len(optimal_assignment[0])):
            # todo change y
            assigned_location = locations[optimal_assignment[1][i]]
            movement[robots[optimal_assignment[0][i]]].append(assigned_location)

        for robot in robots:
            robot.set_movement(movement[robot])

    def __str__(self):
        return 'StaticLinePlanner'
