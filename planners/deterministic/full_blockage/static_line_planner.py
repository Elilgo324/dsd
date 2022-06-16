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
        makespan_per_h = {h: farthest_robot.loc.distance_to(Point(farthest_x, h)) / fv for h in H}

        # num of disabled agents
        num_disabled_per_h = {h: sum([1 if a.y + makespan_per_h[h] * v < h else 0 for a in agents]) for h in H}

        # calculate line score
        def damage_score(h):
            return sum([b - agent.y for agent in agents]) - num_disabled_per_h[h] * (b - h)

        damage_score_per_h = {h: damage_score(h) for h in H}

        # assign robots on h opt
        h_opt = min(H, key=lambda h: damage_score_per_h[h])

        # combine optimal movements
        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[optimal_assignment[0][i]]
            movement[assigned_robot].append(Point(optimal_x[assigned_robot], h_opt))

        completion_time = 0
        if num_disabled_per_h[h_opt] > 0:
            completion_time = (h_opt - y_min_a) / v

        return movement, \
               completion_time, \
               damage_score_per_h[h_opt], \
               num_disabled_per_h[h_opt]

    def __str__(self):
        return 'StaticLinePlanner'
