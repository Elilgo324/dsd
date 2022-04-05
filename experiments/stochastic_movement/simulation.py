import json

from environment.agents.fixed_velocity_agent import FixedVelocityAgent
from environment.agents.stochastic_agent import StochasticAgent
from environment.stochastic_environment import StochasticEnvironment
from planners.partial_blockage.additive_static_lack_planner import AdditiveStaticLackPlanner
from planners.planner import Planner
from planners.stochastic_movement.stochastic_static_lack_planner import StochasticStaticLackPlanner
from utils.functions import *

with open('./dev_config.json') as json_file:
    config = json.load(json_file)


def run(planner: Planner) -> None:
    agents = [StochasticAgent(sample_point(config['x_buffer'], config['x_buffer'] + config['x_size'],
                                              config['y_buffer'], config['y_buffer'] + config['y_size_init']),
                                 config['agent_speed'], config['advance_distribution']) for _ in range(config['num_agents'])]

    robots = [BasicRobot(sample_point(config['x_buffer'], config['x_size'] + config['x_buffer'], config['y_buffer'], config['y_buffer'] + config['y_size_init']),
                         config['robot_speed'], config['disablement_range'], has_mode=True)
              for _ in range(config['num_robots'])]

    env = StochasticEnvironment(agents=agents, robots=robots, top_border=config['y_size']+config['y_buffer'], right_border=config['x_size'] + config['x_buffer'], left_border=config['x_buffer'])

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
    planners = [StochasticStaticLackPlanner() for _ in range(1)]
    for planner in planners:
        print(f'running {str(planner)} ..')
        run(planner)
