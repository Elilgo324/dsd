import os
from datetime import datetime
from math import sqrt
from random import uniform
from typing import List

import imageio as imageio
import matplotlib.pyplot as plt

from agents.base_agent import BaseAgent
from environment import Environment
from utils.point import Point
from robots.basic_robot import BasicRobot


def sample_point(x_min: float, x_max: float, y_min: float, y_max: float) -> Point:
    return Point(uniform(x_min, x_max), uniform(y_min, y_max))


def meeting_height(robot: BasicRobot, agent: BaseAgent) -> float:
    f = robot.fv / agent.v
    a, b = robot.x, robot.y
    c, d = agent.x, agent.y
    inside_sqrt = a ** 2 * f ** 2 - a ** 2 - 2 * a * c * f ** 2 + 2 * a * c \
                  + b ** 2 * f ** 2 - 2 * b * d * f ** 2 + c ** 2 * f ** 2 - c ** 2 + d ** 2 * f ** 2
    num = sqrt(inside_sqrt) - b + d * f ** 2
    den = f ** 2 - 1
    h = num / den

    if h < agent.y:
        raise ValueError('meeting height cannot be lower than agent')

    return h


def plot_environment(robots: List[BasicRobot], agents: List[BaseAgent], env: Environment, config) -> None:
    plt.clf()
    plt.xlim(0, config['x_size'] + 2 * config['x_buffer'])
    plt.ylim(0, config['y_size'] + 2 * config['y_buffer'])
    plt.plot([config['x_buffer'], config['x_buffer'], config['x_buffer'] + config['x_size'], config['x_buffer'] +
              config['x_size'], config['x_buffer']],
             [config['y_buffer'], config['y_buffer'] + config['y_size'], config['y_buffer'] + config['y_size'],
              config['y_buffer'], config['y_buffer']], c='black')
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


def write_report(planner: str,
                 num_agents: int,
                 num_robots: int,
                 f: float,
                 active_time: float,
                 planner_time: float,
                 damage: float,
                 num_disabled: int) -> None:
    stats = [planner, num_agents, num_robots, f, active_time, planner_time, damage, num_disabled]

    file_name = 'results.csv'
    if not os.path.exists(file_name):
        file = open('results.csv', 'a+')
        file.write('planner,num_agents,num_robots,f,active_time,planner_time,damage,'
                   'num_disabled\n')
    else:
        file = open('results.csv', 'a+')

    file.write(",".join([str(s) for s in stats]))
    file.write('\n')
