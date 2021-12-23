import os
from datetime import datetime
from random import uniform
from typing import List

import imageio as imageio
import matplotlib.pyplot as plt

from agents.base_agent import BaseAgent
from environment import Environment
from robots.basic_robot import BasicRobot
from utils.point import Point


def sample_point(x_min: float, x_max: float, y_min: float, y_max: float) -> Point:
    return Point(uniform(x_min, x_max), uniform(y_min, y_max))


def plot_environment(robots: List[BasicRobot], agents: List[BaseAgent], env: Environment, config) -> None:
    buffer = config['buffer']
    X_SIZE, Y_SIZE = config['x_size'], config['y_size']
    plt.clf()
    plt.xlim(-buffer, X_SIZE + buffer)
    plt.ylim(-buffer, Y_SIZE + buffer)
    plt.plot([0, 0, X_SIZE, X_SIZE, 0], [0, Y_SIZE, Y_SIZE, 0, 0], c='black')
    plt.scatter([r.x for r in robots], [r.y for r in robots], c='blue')
    for i in range(len(robots)):
        plt.annotate(i, (robots[i].x, robots[i].y))
    plt.scatter([a.x for a in agents], [a.y for a in agents], c='red')
    plt.title(env.stats(), fontsize=10)
    plt.savefig(f'./plots/{env.step}')


def create_gif_from_plots(prefix=''):
    filenames = os.listdir('./plots/')
    filenames = [file[:-4] for file in filenames]
    with imageio.get_writer(f'./gifs/{prefix}-{datetime.now().minute}.gif', mode='I') as writer:
        for filename in sorted(filenames, key=int):
            image = imageio.imread('./plots/' + filename + '.png')
            writer.append_data(image)

    # Remove files
    for filename in filenames:
        os.remove('./plots/' + filename + '.png')


def write_report(planner: str, config, env: Environment, completion_time: float, planning_time: float):
    # planner
    planner = planner

    # settings
    num_agents = config['num_agents']
    num_robots = len(env.robots)
    f = config['robot_speed'] / config['agent_speed']

    # times
    overall_time = env.step
    completion_time = completion_time
    planner_time = planning_time

    # disablement
    damage = env.acc_damage
    num_disabled = env.agents_disabled
    num_escaped = env.agents_escaped

    stats = [planner, num_agents, num_robots, f, overall_time, completion_time, planner_time, damage,
             num_disabled, num_escaped]

    file_name = 'results.csv'
    if not os.path.exists(file_name):
        file = open('results.csv', 'a+')
        file.write('planner,num_agents,num_robots,f,overall_time,completion_time,planner_time,damage,'
                   'num_disabled,num_escaped\n')
    else:
        file = open('results.csv', 'a+')

    file.write(",".join([str(s) for s in stats]))
    file.write('\n')
