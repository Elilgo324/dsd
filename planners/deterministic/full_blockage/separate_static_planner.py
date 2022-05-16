from typing import Dict, Tuple

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class SeparateStaticPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents = env.agents
        movement = {robot: [] for robot in robots}

        v = agents[0].v
        fv = robots[0].fv
        b = env.border

        x_min_a = min(a.x for a in agents)
        x_max_a = max(a.x for a in agents)
        bucket_size = (x_max_a - x_min_a) / len(robots)
        agents_buckets = [[] for _ in range(len(robots))]

        # divide into buckets
        for a in agents:
            bucket_id = floor((a.x - x_min_a) / bucket_size)
            if bucket_id < len(agents_buckets):
                agents_buckets[bucket_id].append(a)
            else:
                agents_buckets[-1].append(a)

        # final x locations
        opt_x = [x_min_a + bucket_size * (i + 0.5) for i in range(len(robots))]

        # create trp scores
        distances = [[] for _ in range(len(robots))]
        data_per_robot_bucket = {i_robot: {i_bucket: None for i_bucket in range(len(agents_buckets))}
                                     for i_robot in range(len(robots))}

        for i_robot in range(len(robots)):
            robot = robots[i_robot]
            for i_bucket in range(len(agents_buckets)):
                bucket = agents_buckets[i_bucket]

                if len(bucket) == 0:
                    distances[i_robot].append(0)
                    data_per_robot_bucket[i_robot][i_bucket] = {'completion_time': 0, 'num_disabled': 0, 'h_opt': 0}
                    continue

                H = [meeting_height(robot, BaseAgent(Point(opt_x[i_bucket], agent.y), agent.v)) for agent in bucket]

                makespan_per_h = {h: robot.loc.distance_to(Point(opt_x[i_bucket], h)) / fv for h in H}
                num_disabled_per_h = {h: sum([1 if a.y + makespan_per_h[h] * v < h else 0 for a in bucket]) for h in H}

                def damage_score(h):
                    return sum([b - agent.y for agent in bucket]) - num_disabled_per_h[h] * (b - h)

                damage_score_per_h = {h: damage_score(h) for h in H}
                h_opt = min(H, key=lambda h: damage_score_per_h[h])

                distances[i_robot].append(damage_score_per_h[h_opt])

                completion_time = 0
                if num_disabled_per_h[h_opt] > 0:
                    completion_time = (h_opt - min([a.y for a in bucket])) / v
                data_per_robot_bucket[i_robot][i_bucket] = {'completion_time': completion_time,
                                                            'num_disabled': num_disabled_per_h[h_opt],
                                                            'h_opt': h_opt}

        optimal_assignment = linear_sum_assignment(distances)

        num_disabled = 0
        completion_time = 0
        acc_damage = 0

        for i_robot in range(len(optimal_assignment[0])):
            assigned_robot = robots[i_robot]
            i_bucket = optimal_assignment[1][i_robot]
            movement[assigned_robot] = [Point(opt_x[i_bucket], data_per_robot_bucket[i_robot][i_bucket]['h_opt'])]
            num_disabled += data_per_robot_bucket[i_robot][i_bucket]['num_disabled']
            completion_time = max(completion_time, data_per_robot_bucket[i_robot][i_bucket]['completion_time'])
            acc_damage += distances[i_robot][i_bucket]

        return movement, completion_time, acc_damage, num_disabled

    def __str__(self):
        return 'SeparateStaticPlanner'
