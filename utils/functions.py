import os
from datetime import datetime
from random import uniform
from typing import List

import imageio as imageio
import matplotlib.pyplot as plt

from agents.base_agent import BaseAgent
from environment import Environment
from robots.basic_robot import BasicRobot
from utils.consts import Consts
from utils.point import Point


def sample_point(x_min: float, x_max: float, y_min: float, y_max: float) -> Point:
    return Point(uniform(x_min, x_max), uniform(y_min, y_max))


def plot_environment(robots: List[BasicRobot], agents: List[BaseAgent], env: Environment) -> None:
    buffer = 10
    X_SIZE,Y_SIZE = env.world_size
    plt.clf()
    plt.xlim(-buffer, X_SIZE + buffer)
    plt.ylim(-buffer, Y_SIZE + buffer)
    plt.plot([0, 0, X_SIZE, X_SIZE, 0], [0, Y_SIZE, Y_SIZE, 0, 0], c='black')
    plt.scatter([r.x for r in robots], [r.y for r in robots], c='blue')
    for i in range(len(robots)):
        plt.annotate(i, (robots[i].x, robots[i].y))
    plt.scatter([a.x for a in agents], [a.y for a in agents], c='red')
    plt.title(env.stats(), fontsize=10)
    plt.savefig(f'../plots/{env.step}')


def create_gif_from_plots(prefix=''):
    filenames = os.listdir('../plots/')
    filenames = [file[:-4] for file in filenames]
    with imageio.get_writer(f'../gifs/{prefix}-{datetime.now().minute}.gif', mode='I') as writer:
        for filename in sorted(filenames, key=int):
            image = imageio.imread('../plots/' + filename + '.png')
            writer.append_data(image)

    # Remove files
    for filename in filenames:
        os.remove('../plots/' + filename + '.png')


def write_report():
    pass
