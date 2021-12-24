from typing import Dict, Tuple

from scipy.optimize import linear_sum_assignment

from planners.planner import Planner
from utils.functions import *


class IterativeAssignmentPlanner(Planner):
    def plan(self, env: Environment) -> Tuple[Dict[BasicRobot, List[Point]], float]:
        robots = env.robots
        agents_copy = [a.clone() for a in env.agents]
        movement = {robot: [] for robot in robots}
        free_time = {robot: 0 for robot in robots}

        while len(agents_copy) > 0:
            distances = [[] for _ in range(len(robots))]
            for i in range(len(robots)):
                for a in agents_copy:
                    x_meeting = a.x
                    agent_at_time = BaseAgent(Point(a.x, a.y + free_time[robots[i]] * a.v), a.v)
                    y_meeting = meeting_height(robots[i], agent_at_time)
                    distances[i].append(robots[i].loc.distance_to(Point(x_meeting, y_meeting)))

            optimal_assignment = linear_sum_assignment(distances)
            assigned_robots = optimal_assignment[0]
            assigned_agents = optimal_assignment[1]

            for i in range(len(assigned_robots)):
                assigned_robot = robots[assigned_robots[i]]
                assigned_agent = agents_copy[assigned_agents[i]]

                prev_loc = assigned_robot.loc
                if len(movement[assigned_robot]) > 0:
                    prev_loc = movement[assigned_robot][-1]

                x_meeting = assigned_agent.x
                y_meeting = meeting_height(assigned_robot, assigned_agent)
                meeting_point = Point(x_meeting, y_meeting)

                movement[assigned_robot].append(meeting_point)
                free_time[assigned_robot] += prev_loc.distance_to(meeting_point) / assigned_robot.fv

            agents_to_remove = [agents_copy[i] for i in assigned_agents]
            for a in agents_to_remove:
                agents_copy.remove(a)

        for r in robots:
            if len(movement[r]) > 0:
                movement[r].append(Point(movement[r][-1].x, env.border))

        return movement, max(free_time.values())

    def __str__(self):
        return 'IterativeAssignmentPlanner'
