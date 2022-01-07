from typing import Dict, Tuple

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class SeparateTravelingPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, int]:
        robots = env.robots
        agents = env.agents
        movement = {robot: [] for robot in robots}

        v = agents[0].v
        fv = robots[0].fv

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

        trp_data_per_bucket = {robot: {i: [] for i in range(len(agents_buckets))} for robot in robots}

        # final x locations
        opt_x = [x_min_a + bucket_size * (i + 0.5) for i in range(len(robots))]

        # create trp scores
        distances = [[] for _ in range(len(robots))]
        for i_robot in range(len(robots)):
            robot = robots[i_robot]
            for i_bucket in range(len(agents_buckets)):
                bucket = agents_buckets[i_bucket]

                if len(bucket) == 0:
                    distances[i_robot].append(0)
                    continue

                H = [meeting_height(robot, BaseAgent(Point(opt_x[i_bucket], agent.y), agent.v)) for agent in bucket]

                h_makespan = {h: robot.loc.distance_to(Point(opt_x[i_bucket], h)) / fv for h in H}
                h_trpv = {h: line_trpv(h, fv, bucket, h_makespan[h]) for h in H}

                def damage_score(h):
                    return h_trpv[h]['damage'] + (len(bucket) * h_makespan[h]) / fv

                hs_damage_scores = {h: damage_score(h) for h in H}

                # assign robots on h opt
                h_opt = min(H, key=lambda h: hs_damage_scores[h])

                distances[i_robot].append(hs_damage_scores[h_opt])

                h_trpv[h_opt]['ys'] = [h_opt] + h_trpv[h_opt]['ys']
                h_trpv[h_opt]['t'] = h_makespan[h_opt] + h_trpv[h_opt]['t']
                trp_data_per_bucket[robot][i_bucket] = h_trpv[h_opt]

        optimal_assignment = linear_sum_assignment(distances)

        num_disabled = 0
        active_time = 0
        acc_damage = 0
        for i_robot in range(len(optimal_assignment[0])):
            assigned_robot = robots[i_robot]
            i_bucket = optimal_assignment[1][i_robot]
            Point(opt_x[i_robot], 0)
            movement[assigned_robot] = \
                [Point(opt_x[i_bucket], y) for y in trp_data_per_bucket[assigned_robot][i_robot]['ys']]

            num_disabled += len(movement[assigned_robot])
            active_time = max(active_time, trp_data_per_bucket[assigned_robot][i_bucket]['t'])
            acc_damage += distances[i_robot][i_bucket]

        return movement, active_time, acc_damage, num_disabled

    def __str__(self):
        return 'SeparateTravelingPlanner'
