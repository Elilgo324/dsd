import json
import time
from math import ceil

from agents.fixed_velocity_agent import FixedVelocityAgent
from planners.full_blockage.iterative_assignment_planner import IterativeAssignmentPlanner
from planners.planner import Planner
from utils.functions import *

with open('config.json') as json_file:
    config = json.load(json_file)


def run(planner: Planner):
    agents = [FixedVelocityAgent(sample_point(config['x_buffer'], config['x_buffer'] + config['x_size'],
                                              config['y_buffer'], config['y_buffer'] + config['y_size_init']),
                                 config['agent_speed']) for _ in range(config['num_agents'])]

    x_min = min([a.x for a in agents])
    x_max = max([a.x for a in agents])

    num_robots_for_full_blockage = ceil((x_max - x_min) / (2 * config['disablement_range']))
    robots = [BasicRobot(sample_point(0, config['x_size'] + 2 * config['x_buffer'], 0, config['y_buffer']),
                         config['robot_speed'], config['disablement_range'])
              for _ in range(num_robots_for_full_blockage)]

    env = Environment(agents=agents, robots=robots, border=config['y_size'] + config['y_buffer'])

    before = time.time()
    movement, completion_time = planner.plan(env)
    planning_time = time.time() - before

    for r in robots:
        r.set_movement(movement[r])

    is_finished = False
    while not is_finished:
        plot_environment(robots, agents, env, config)
        is_finished = env.advance()
    plot_environment(robots, agents, env, config)

    create_gif_from_plots(prefix=str(planner))
    write_report(planner=str(planner), config=config, env=env, completion_time=completion_time,
                 planning_time=planning_time)

    print(f'*** results of {str(planner)} ***')
    print(env.stats())


if __name__ == '__main__':
    planners = [IterativeAssignmentPlanner() for _ in range(1)]
    for planner in planners:
        print(f'running {str(planner)} ..')
        run(planner)
