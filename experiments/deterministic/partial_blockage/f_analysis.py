import json
import time
from random import seed

from planners.deterministic.partial_blockage.static_line_lack_planner import StaticLineLackPlanner
from planners.planner import Planner
from utils.functions import *
from world.agents.deterministic_agent import DeterministicAgent

with open('config.json') as json_file:
    config = json.load(json_file)


def run(planner: Planner):
    agents = [DeterministicAgent(sample_point(config['x_buffer'], config['x_buffer'] + config['x_size'],
                                              config['y_buffer'], config['y_buffer'] + config['y_size_init']),
                                 config['agent_speed']) for _ in range(config['num_agents'])]

    robots = [BasicRobot(sample_point(0, config['x_size'] + 2 * config['x_buffer'], 0, config['y_buffer']),
                         config['robot_speed'], config['disablement_range'])
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
                 active_or_copmletion_time=completion_time,
                 planner_time=planning_time,
                 damage=expected_damage,
                 num_disabled=expected_num_disabled,
                 file_name='f_results.csv',
                 is_active_time=False)


if __name__ == '__main__':
    # planners = [PracticalStaticLineLacklPlanner(), IterativeAssignmentPlanner(),
    #             KmeansAssignmentPlanner(), StaticLineLackPlanner()]
    planners = [StaticLineLackPlanner()]

    config['num_agents'] = 200
    for planner in planners:
        for v in [1.4, 1.6, 1.8]:
            # for v in [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2]:
            print(f'*** *** v={v} *** ***')
            for s in range(30):
                seed(s)

                config['robot_speed'] = v
                print(f'running {str(planner)} with seed {s}..')
                run(planner)
