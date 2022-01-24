from planners.planner import Planner
from utils.functions import *


class SeparateStaticLackPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float, float, float, int]:
        robots = env.robots
        agents = env.agents
        b = env.border
        v = agents[0].v
        fv = robots[0].fv

        movement = {robot: [] for robot in robots}
        completion_time = 0
        acc_damage = 0
        num_disabled = 0

        # divide into buckets
        x_min_a = min(a.x for a in agents)
        x_max_a = max(a.x for a in agents)
        bucket_size = (x_max_a - x_min_a) / len(robots)
        agents_buckets = [[] for _ in range(len(robots))]

        for a in agents:
            bucket_id = floor((a.x - x_min_a) / bucket_size)
            if bucket_id < len(agents_buckets):
                agents_buckets[bucket_id].append(a)
            else:
                agents_buckets[-1].append(a)

        # create flow data for each robot and bucket
        score_per_robot_bucket = [[] for _ in range(len(robots))]
        data_per_robot_bucket = {i_robot: {i_bucket: {} for i_bucket in range(len(agents_buckets))}
                                 for i_robot in range(len(robots))}

        for i_robot in range(len(robots)):
            robot = robots[i_robot]

            for i_bucket in range(len(agents_buckets)):
                bucket = agents_buckets[i_bucket]

                if len(bucket) == 0:
                    score_per_robot_bucket[i_robot].append(0)
                    data_per_robot_bucket[i_robot][i_bucket] = {'movement': [],
                                                                'completion_time': 0,
                                                                'damage': 0,
                                                                'num_disabled': 0}
                    continue

                H = [meeting_height(robot, agent) for agent in bucket]
                flow_per_h = {h: flow_moves([robot], bucket, h) for h in H}

                def damage_score(h):
                    non_escaping = flow_per_h[h]['disabled']
                    damage = sum([b - agent.y for agent in agents])
                    damage -= sum([b - h for _ in non_escaping])

                    return damage

                damage_score_per_h = {h: damage_score(h) for h in H}
                h_opt = min(H, key=lambda h: damage_score_per_h[h])

                score_per_robot_bucket[i_robot].append(damage_score_per_h[h_opt])

                flow_per_h[h_opt]['movement'] = flow_per_h[h_opt]['movement'][robot]
                data_per_robot_bucket[i_robot][i_bucket]['movement'] = flow_per_h[h_opt]['movement']

                cur_completion_time = 0
                if len(flow_per_h[h_opt]['disabled']) > 0:
                    cur_completion_time = (h_opt - min([agent.y for agent in flow_per_h[h_opt]['disabled']])) / v
                data_per_robot_bucket[i_robot][i_bucket]['completion_time'] = cur_completion_time

                data_per_robot_bucket[i_robot][i_bucket]['damage'] = damage_score_per_h[h_opt]
                data_per_robot_bucket[i_robot][i_bucket]['num_disabled'] = len(flow_per_h[h_opt]['movement'])

        # combine optimal results
        optimal_assignment = linear_sum_assignment(score_per_robot_bucket)

        for i_robot in range(len(optimal_assignment[0])):
            assigned_robot = robots[i_robot]
            i_bucket = optimal_assignment[1][i_robot]

            movement[assigned_robot] = data_per_robot_bucket[i_robot][i_bucket]['movement']
            completion_time = max(completion_time, data_per_robot_bucket[i_robot][i_bucket]['completion_time'])
            acc_damage += data_per_robot_bucket[i_robot][i_bucket]['damage']
            num_disabled += data_per_robot_bucket[i_robot][i_bucket]['num_disabled']

        return movement, -1, completion_time, acc_damage, num_disabled

    def __str__(self):
        return 'SeparateStaticLackPlanner'
