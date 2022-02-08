import json
import time
from math import ceil
from random import seed

from planners.full_blockage.separate_traveling_planner import SeparateTravelingPlanner
from planners.full_blockage.static_line_planner import StaticLinePlanner
from planners.full_blockage.traveling_line_planner import TravelingLinePlanner
from planners.baseline.kmeans_assignment_planner import KmeansAssignmentPlanner
from planners.full_blockage.low_traveling_line_planner import LowTravelingLinePlanner
from planners.baseline.iterative_assignment_planner import IterativeAssignmentPlanner
from planners.planner import Planner
from utils.functions import *

with open('./config.json') as json_file:
    config = json.load(json_file)


def run(planner: Planner):
    agents = [FixedVelocityAgent(sample_point(config['x_buffer'], config['x_buffer'] + config['x_size'],
                                              config['y_buffer'], config['y_buffer'] + config['y_size_init']),
                                 config['agent_speed']) for _ in range(config['num_agents'])]

    x_min = min([a.x for a in agents])
    x_max = max([a.x for a in agents])

    num_robots_for_full_blockage = ceil((x_max - x_min) / (2 * config['disablement_range']))
    robots = [BasicRobot(sample_point(0, config['x_size'] + 2 * config['x_buffer'], 0, config['y_buffer']),
                         config['robot_speed'], config['disablement_range'], has_mode=True)
              for _ in range(num_robots_for_full_blockage)]

    env = Environment(agents=agents, robots=robots, border=config['y_size'] + config['y_buffer'])

    before = time.time()
    movement, completion_time, expected_damage, expected_num_disabled = planner.plan(env)
    planning_time = time.time() - before

    write_report(planner=str(planner),
                 num_agents=config['num_agents'],
                 num_robots=num_robots_for_full_blockage,
                 f=config['robot_speed'] / config['agent_speed'],
                 d=config['disablement_range'],
                 completion_time=completion_time,
                 planner_time=planning_time,
                 damage=expected_damage,
                 num_disabled=expected_num_disabled,
                 file_name='d_results.csv')


if __name__ == '__main__':
    planners = [SeparateTravelingPlanner(),
                TravelingLinePlanner()]

    config['num_agents'] = 300

    for planner in planners:
        for v in [3, 5, 10, 15]:
            print(f'*** *** v={v} *** ***')
            for s in range(30):
                seed(s)

                config['disablement_range'] = v
                print(f'running {str(planner)} with seed {s}..')
                run(planner)
