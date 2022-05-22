import json
from math import ceil
from random import seed

from world.agents.deterministic_agent import DeterministicAgent
from planners.deterministic.baseline.kmeans_assignment_planner import KmeansAssignmentPlanner
from planners.planner import Planner
from utils.functions import *

with open('config.json') as json_file:
    config = json.load(json_file)


def run(planner: Planner):
    config['num_agents'] = 20
    agents = [DeterministicAgent(sample_point(config['x_buffer'], config['x_buffer'] + config['x_size'],
                                              config['y_buffer'], config['y_buffer'] + config['y_size_init']),
                                 config['agent_speed']) for _ in range(config['num_agents'])]

    x_min = min([a.x for a in agents])
    x_max = max([a.x for a in agents])

    num_robots_for_full_blockage = ceil((x_max - x_min) / (2 * config['disablement_range']))
    num_robots_for_full_blockage = 2
    robots = [BasicRobot(sample_point(0, config['x_size'] + 2 * config['x_buffer'], 0, config['y_buffer']),
                         config['robot_speed'], config['disablement_range'])
              for _ in range(num_robots_for_full_blockage)]

    env = Environment(agents=agents, robots=robots, border=config['y_size'] + config['y_buffer'])

    movement, _, _, _ = planner.plan(env)

    for r in robots:
        r.set_movement(movement[r])

    is_finished = False
    while not is_finished:
        plot_environment(robots, agents, env, config)
        is_finished = env.advance()
    plot_environment(robots, agents, env, config)

    create_gif_from_plots(prefix=str(planner))

    print(f'*** results of {str(planner)} ***')
    print(env.stats())


if __name__ == '__main__':
    seed(39)

    # planners = [RandomWalk10Planner(), OfflineChasingPlanner(), OnlineChasingPlanner(), StaticLinePlanner()]
    planners = [KmeansAssignmentPlanner() for _ in range(1)]
    for planner in planners:
        print(f'running {str(planner)} ..')
        run(planner)
