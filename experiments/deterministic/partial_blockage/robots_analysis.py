import json
import time
from random import seed

from world.agents.deterministic_agent import DeterministicAgent
from planners.deterministic.baseline.iterative_assignment_planner import IterativeAssignmentPlanner
from planners.deterministic.baseline.kmeans_assignment_planner import KmeansAssignmentPlanner
from planners.deterministic.partial_blockage.additive_static_lack_planner import AdditiveStaticLackPlanner
from planners.deterministic.partial_blockage.static_line_lack_planner import StaticLineLackPlanner
from planners.planner import Planner
from utils.functions import *

with open('config.json') as json_file:
    config = json.load(json_file)


def run(planner: Planner):
    agents = [DeterministicAgent(sample_point(config['x_buffer'], config['x_buffer'] + config['x_size'],
                                              config['y_buffer'], config['y_buffer'] + config['y_size_init']),
                                 config['agent_speed']) for _ in range(config['num_agents'])]

    robots = [BasicRobot(sample_point(0, config['x_size'] + 2 * config['x_buffer'], 0, config['y_buffer']),
                         config['robot_speed'], config['disablement_range'], has_mode=True)
              for _ in range(config['num_robots'])]

    env = Environment(agents=agents, robots=robots, border=config['y_size'] + config['y_buffer'])

    before = time.time()
    movement, completion_time, expected_damage, expected_num_disabled = planner.plan(env)
    planning_time = time.time() - before

    write_report(planner=str(planner),
                 num_agents=config['num_agents'],
                 num_robots=config['num_robots'],
                 f=config['robot_speed'] / config['agent_speed'],
                 d=config['disablement_range'],
                 completion_time=completion_time,
                 planner_time=planning_time,
                 damage=expected_damage,
                 num_disabled=expected_num_disabled,
                 file_name='robots_results.csv')


if __name__ == '__main__':
    planners = [AdditiveStaticLackPlanner()]
    # planners = [AdditiveStaticLackPlanner(), SeparateStaticLackPlanner(), IterativeAssignmentPlanner(), KmeansAssignmentPlanner()]
    planners = [IterativeAssignmentPlanner(), KmeansAssignmentPlanner()]
    planners = [StaticLineLackPlanner()]

    config['num_agents'] = 300

    for planner in planners:
        # for v in [2, 3, 4, 5, 6, 7, 8]:
        for v in [3, 4, 5, 6, 7, 8]:
            print(f'*** *** v={v} *** ***')
            for s in range(10):
                seed(s)

                config['num_robots'] = v
                print(f'running {str(planner)} with seed {s}..')
                run(planner)
