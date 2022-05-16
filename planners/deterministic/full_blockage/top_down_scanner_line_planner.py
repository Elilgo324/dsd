from typing import Tuple, Dict

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class TopDownScannerPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents = [a.clone() for a in env.agents]
        agents = sorted(agents, key=lambda a: a.y, reverse=True)

        v = agents[0].v
        fv = robots[0].fv
        r = robots[0].d

        x_min_a = min(a.x for a in agents)
        x_max_a = max(a.x for a in agents)
        actual_range = (x_max_a - x_min_a) / len(robots)
        y_min_a = agents[0].y

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

        # first line meeting the highest agent
        construction_height = meeting_height(farthest_robot,
                                             BaseAgent(loc=Point(farthest_x, agents[0].y), v=v))
        makespan = farthest_robot.loc.distance_to(Point(farthest_x, construction_height)) / fv

        movement = {robot: [Point(optimal_x[robot], construction_height)] for robot in robots}
        completion_time = makespan
        acc_damage = completion_time * len(agents) * v
        num_disabled = 1

        prev_height = construction_height
        for i_agent in range(1, len(agents)):
            agent = agents[i_agent]
            meeting_h = meeting_height(BasicRobot(loc=Point(0, construction_height), r=r, fv=fv),
                                       BaseAgent(loc=Point(0, agent.y + makespan * v), v=v))
            cur_completion_time = (prev_height - meeting_h) / fv
            prev_height = meeting_h
            for robot in robots:
                movement[robot].append(Point(optimal_x[robot], meeting_h))

            completion_time += cur_completion_time
            acc_damage += (len(agents) - i_agent - 1) * cur_completion_time * v
            num_disabled += 1

        return movement, completion_time, acc_damage, num_disabled

    def __str__(self):
        return 'TopDownScannerPlanner'
