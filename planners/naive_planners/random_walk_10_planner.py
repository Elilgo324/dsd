from planners.planner import Planner
from utils.functions import *


class RandomWalk10Planner(Planner):
    def plan(self, env: Environment) -> None:
        robots = env.robots

        movement = {robot: [] for robot in robots}

        x_max, y_max = env.world_size

        for _ in range(10):
            for robot in robots:
                movement[robot].append(sample_point(0, x_max, 0, y_max))

        for robot in robots:
            robot.set_movement(movement[robot])

    def __str__(self):
        return 'RandomWalk10Planner'
