import json
import time
from math import ceil

from agents.fixed_velocity_agent import FixedVelocityAgent
from planners.full_blockage.static_line_planner import StaticLinePlanner
from planners.planner import Planner
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
    robots = [BasicRobot(sample_point(-config['buffer'], config['x_size'] + config['buffer'], 0, y_min),
                         config['robot_speed'], config['disablement_range'])
              for _ in range(num_robots_for_full_blockage)]

    env = Environment(agents=agents, robots=robots, world_size=(config['x_size'], config['y_size']))

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
    # planners = [RandomWalk10Planner(), OfflineChasingPlanner(), OnlineChasingPlanner(), StaticLinePlanner()]
    planners = [StaticLinePlanner() for _ in range(5)]
    for planner in planners:
        print(f'running {str(planner)} ..')
        run(planner)
