import json
import time
from math import ceil

from agents.fixed_velocity_agent import FixedVelocityAgent
from planners.full_blockage.traveling_line_planner import TravelingLinePlanner
from planners.planner import Planner
from robots.waiting_robot import WaitingRobot
from utils.functions import *

with open('config.json') as json_file:
    config = json.load(json_file)


def run(planner: Planner):
    agents = [FixedVelocityAgent(sample_point(0, config['x_size'], 0.2 * config['y_size_init'], config['y_size_init']),
                                 config['agent_speed']) for _ in range(config['num_agents'])]

    y_min = min([a.y for a in agents])
    x_min = min([a.x for a in agents])
    x_max = max([a.x for a in agents])
    num_robots_for_full_blockage = ceil((x_max - x_min) / (2 * config['disablement_range']))
    robots = [WaitingRobot(sample_point(-config['buffer'], config['x_size'] + config['buffer'], 0, y_min),
                           config['robot_speed'], config['disablement_range'])
              for _ in range(num_robots_for_full_blockage)]

    env = Environment(agents=agents, robots=robots, world_size=(config['x_size'], config['y_size']))

    before = time.time()
    movement, completion_time = planner.plan(env)
    planning_time = time.time() - before

    makespan_time = max([movement[r][0].distance_to(r.loc) / config['robot_speed'] for r in robots])
    for r in robots:
        r.set_movement(movement[r])
        r.set_wait_time(makespan_time)

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
    planners = [TravelingLinePlanner() for _ in range(1)]
    for planner in planners:
        print(f'running {str(planner)} ..')
        run(planner)
