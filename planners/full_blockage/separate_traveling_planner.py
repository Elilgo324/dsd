from math import floor
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
                H = [meeting_height(robot, BaseAgent(Point(opt_x[i_bucket], agent.y), agent.v)) for agent in bucket]

                h_makespan = {h: robot.loc.distance_to(Point(opt_x[i_bucket], h)) for h in H}
                h_trpv = {h: line_trpv(h, fv, bucket, h_makespan[h]) for h in H}

                def damage_score(h):
                    score = h_trpv[h]['damage']
                    return score + (len(bucket) * makespan(h)) / fv

                hs_damage_scores = {h: damage_score(h) for h in H}

                # assign robots on h opt
                h_opt = min(H, key=lambda h: hs_damage_scores[h])

                distances[i_robot].append(hs_damage_scores[h_opt])

                trp_data = h_trpv[h_opt]
                trp_data['ys'] = [h_opt] + trp_data['ys']
                trp_data_per_bucket[robot][i_bucket] = trp_data

        optimal_assignment = linear_sum_assignment(distances)
        for i in range(len(optimal_assignment[0])):
            assigned_robot = robots[i]
            i_bucket = optimal_assignment[1][i]
            movement[assigned_robot] = [Point(opt_x[i_bucket], y) for y in trp_data_per_bucket[assigned_robot][i]['ys']]

        num_disabled = 0
        active_time = 0
        acc_damage = 0
        for i_robot in range(len(optimal_assignment[0])):
            robot = robots[i_robot]
            i_bucket = optimal_assignment[1][i_robot]

            def makespan(h):
                return sqrt((opt_x[i_bucket] - robot.x) ** 2 + (h - robot.y) ** 2) / robot.fv

            ys = trp_data_per_bucket[robot][i_bucket]['ys']
            num_disabled += len(ys)
            active_time = max(active_time, makespan(ys[0]) + trp_data_per_bucket[robot][i_bucket]['t'])
            acc_damage += distances[i_robot][i_bucket]

        return movement, active_time, active_time, num_disabled

    def __str__(self):
        return 'SeparateTravelingPlanner'
