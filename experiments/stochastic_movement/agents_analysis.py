import json
import time
from random import seed

from environment.agents.deterministic_agent import DeterministicAgent
from environment.agents.stochastic_agent import StochasticAgent
from environment.robots.timing_robot import TimingRobot
from environment.stochastic_environment import StochasticEnvironment
from planners.baseline.iterative_assignment_planner import IterativeAssignmentPlanner
from planners.baseline.kmeans_assignment_planner import KmeansAssignmentPlanner
from planners.partial_blockage.additive_static_lack_planner import AdditiveStaticLackPlanner
from planners.partial_blockage.separate_static_lack_planner import SeparateStaticLackPlanner
from planners.partial_blockage.static_line_lack_planner import StaticLineLackPlanner
from planners.partial_blockage.low_static_line_lack import LowStaticLineLacklPlanner
from planners.planner import Planner
from planners.stochastic_movement.stochastic_additive_planner import StochasticAdditivePlanner
from planners.stochastic_movement.stochastic_static_lack_planner import StochasticStaticLackPlanner
from utils.functions import *

with open('./dev_config.json') as json_file:
    config = json.load(json_file)


def run(planner: Planner):
    agents = [StochasticAgent(sample_point(config['x_buffer'], config['x_buffer'] + config['x_size'],
                                           config['y_buffer'], config['y_buffer'] + config['y_size_init']),
                              config['agent_speed'], config['advance_distribution']) for _ in
              range(config['num_agents'])]

    robots = [TimingRobot(sample_point(0, config['x_size'] + 2 * config['x_buffer'], 0, config['y_buffer']),
                          config['robot_speed'], config['disablement_range']) for _ in range(config['num_robots'])]

    env = StochasticEnvironment(agents=agents, robots=robots, top_border=config['y_size'] + config['y_buffer'],
                                right_border=config['x_size'] + config['x_buffer'], left_border=config['x_buffer'])

    before = time.time()
    _, active_time, expected_damage, expected_num_disabled, _ = planner.plan(env)
    planning_time = time.time() - before

    write_report(planner=str(planner),
                 num_agents=config['num_agents'],
                 num_robots=config['num_robots'],
                 f=config['robot_speed'] / config['agent_speed'],
                 d=config['disablement_range'],
                 active_time=active_time,
                 planner_time=planning_time,
                 damage=expected_damage,
                 num_disabled=expected_num_disabled,
                 file_name='agents_results.csv')


if __name__ == '__main__':
    planners = [StochasticStaticLackPlanner()]

    for planner in planners:
        for v in [400, 500, 600, 700, 800, 900, 1000]:
            print(f'*** *** v={v} *** ***')
            for s in range(3):
                seed(s)

                config['num_agents'] = v
                print(f'running {str(planner)} with seed {s}..')
                run(planner)
