import math
import operator
import os
from datetime import datetime
from math import sqrt, floor
from random import uniform, randint
from typing import List, Tuple, Dict, Union

from scipy.integrate import quad

import matplotlib.pyplot as plt
import scipy.stats

import scipy
import seaborn as sb
import imageio
import numpy as np

from world.agents.base_agent import BaseAgent
from world.environment import Environment
from world.robots.basic_robot import BasicRobot
from utils.point import Point


def plot_environment(robots: List[BasicRobot], agents: List[BaseAgent],
                     env: Environment, config: Dict[str, float]) -> None:
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
    # plt.gca().set_aspect('equal', adjustable='box')
    plt.savefig(f'./plots/{env.step}')


def create_gif_from_plots(prefix: str = '') -> None:
    filenames = os.listdir('./plots/')
    filenames = [file[:-4] for file in filenames]
    with imageio.get_writer(f'./gifs/{prefix}-{datetime.now().minute}.gif', mode='I') as writer:
        for filename in sorted(filenames, key=int):
            image = imageio.imread('./plots/' + filename + '.png')
            writer.append_data(image)

    # remove files
    for filename in filenames:
        os.remove('./plots/' + filename + '.png')


def write_report(planner: str,
                 num_agents: int,
                 num_robots: int,
                 f: float,
                 d: float,
                 active_or_copmletion_time: float,
                 planner_time: float,
                 damage: float,
                 num_disabled: int,
                 file_name: str = 'results.csv', is_active_time=True) -> None:
    stats = [planner, num_agents, num_robots, f, d, active_or_copmletion_time, planner_time, damage, num_disabled]

    if not os.path.exists(file_name):
        file = open(file_name, 'a+')
        if is_active_time:
            file.write('planner,num_agents,num_robots,f,d,active_time,planner_time,damage,'
                       'num_disabled\n')
        else:
            file.write('planner,num_agents,num_robots,f,d,completion_time,planner_time,damage,'
                       'num_disabled\n')
    else:
        file = open(file_name, 'a+')

    file.write(",".join([str(s) for s in stats]))
    file.write('\n')


def sample_point(x_min: float, x_max: float, y_min: float, y_max: float, is_int: bool = False) -> Point:
    if is_int:
        return Point(randint(int(x_min), int(x_max) - 1), randint(int(y_min), int(y_max)))
    return Point(uniform(x_min, x_max), uniform(y_min, y_max))


def meeting_height(robot: BasicRobot, agent: BaseAgent) -> float:
    f = robot.fv / agent.v
    a, b = robot.x, robot.y
    c, d = agent.x, agent.y

    if math.isclose(a, c) and math.isclose(b, d):
        return b

    inside_sqrt = a ** 2 * f ** 2 - a ** 2 - 2 * a * c * f ** 2 + 2 * a * c \
                  + b ** 2 * f ** 2 - 2 * b * d * f ** 2 + c ** 2 * f ** 2 - c ** 2 + d ** 2 * f ** 2
    num = sqrt(inside_sqrt) - b + d * f ** 2
    den = f ** 2 - 1
    h = num / den

    if h < agent.y:
        raise ValueError('meeting height cannot be lower than agent')

    return h


def map_into_2_pows(costs: List[List[float]]) -> List[List[float]]:
    rows_num = len(costs)
    cols_num = len(costs[0])

    enumerate_object = enumerate([item for sublist in costs for item in sublist])
    sorted_pairs = sorted(enumerate_object, key=operator.itemgetter(1))
    sorted_indices = [index for index, element in sorted_pairs]

    pows = 2 ** -int((rows_num * cols_num) / 2)
    for i in sorted_indices:
        row = floor(i / cols_num)
        col = i % cols_num
        costs[row][col] = pows
        pows *= 2

    return costs


def integrate_gauss(mu: float, sigma: float, left: float, right: float) -> float:
    if sigma == 0:
        if left <= mu <= right:
            return 1
        return 0

    def normal_distribution_function(x):
        p = 1 / math.sqrt(2 * math.pi * sigma ** 2)
        return p * np.exp(-0.5 / sigma ** 2 * (x - mu) ** 2)

    stuff = quad(normal_distribution_function, left, right)
    return round(stuff[0], 3)


def future_sigma(sigma: float, stride: int) -> float:
    future_variance = sigma ** 2 * stride
    return future_variance ** 0.5


def sigma_points_per_h(mu: float, init_sigma: float, init_y: float, h: float) -> Tuple[Point,Point]:
    steps = h-init_y
    new_sigma = (steps * init_sigma**2)**0.5
    return Point(mu-new_sigma, h), Point(mu-new_sigma, h)


def distances_from_point():
    pass


