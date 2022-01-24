import json
import time
from random import seed

from planners.baseline.iterative_assignment_planner import IterativeAssignmentPlanner
from planners.baseline.kmeans_assignment_planner import KmeansAssignmentPlanner
from planners.partial_blockage.static_line_lack_planner import StaticLineLackPlanner
from planners.partial_blockage.practical_static_line_lack import PracticalStaticLineLacklPlanner
from planners.planner import Planner
from utils.functions import *

with open('./config.json') as json_file:
    config = json.load(json_file)


def run(planner: Planner):
    agents = [FixedVelocityAgent(sample_point(config['x_buffer'], config['x_buffer'] + config['x_size'],
                                              config['y_buffer'], config['y_buffer'] + config['y_size_init']),
                                 config['agent_speed']) for _ in range(config['num_agents'])]

    robots = [BasicRobot(sample_point(0, config['x_size'] + 2 * config['x_buffer'], 0, config['y_buffer']),
                         config['robot_speed'], config['disablement_range'], has_mode=True)
              for _ in range(config['num_robots'])]

    env = Environment(agents=agents, robots=robots, border=config['y_size'] + config['y_buffer'])

    before = time.time()
    movement, active_time, completion_time, expected_damage, expected_num_disabled = planner.plan(env)
    planning_time = time.time() - before

    write_report(planner=str(planner),
                 num_agents=config['num_agents'],
                 num_robots=config['num_robots'],
                 f=config['robot_speed'] / config['agent_speed'],
                 d=config['disablement_range'],
                 active_time=active_time,
                 completion_time=completion_time,
                 planner_time=planning_time,
                 damage=expected_damage,
                 num_disabled=expected_num_disabled,
                 file_name='robots_results.csv')


if __name__ == '__main__':
    # planners = [IterativeAssignmentPlanner(),
    #             KmeansAssignmentPlanner()]
    planners = [PracticalStaticLineLacklPlanner(), StaticLineLackPlanner()]
    # StaticLineLackPlanner(), PracticalStaticLineLacklPlanner(),

    config['num_agents'] = 200

    for planner in planners:
        for v in [2, 4, 6, 8]:
            print(f'running for v={v} ..')
            for s in range(30):
                seed(s)

                config['num_robots'] = v
                print(f'running {str(planner)} with seed {s} ..')
                run(planner)