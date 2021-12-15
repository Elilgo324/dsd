from random import randint
from typing import List

from agents.base_agent import BaseAgent
from robots.basic_robot import BasicRobot
from utils.consts import Consts
from utils.point import Point

import matplotlib.pyplot as plt


def sample_point() -> Point:
    return Point(randint(*Consts.X_RANGE), randint(*Consts.Y_RANGE))


def sample_point_under(y_limit: int) -> Point:
    return Point(randint(*Consts.X_RANGE), randint(0,y_limit))


def plot_environment(robots: List[BasicRobot], agents: List[BaseAgent]) -> None:
    plt.xlim(*Consts.X_RANGE)
    plt.ylim(*Consts.Y_RANGE)
    plt.scatter([r.x for r in robots], [r.y for r in robots], c='blue')
    plt.scatter([a.x for a in agents], [a.y for a in agents], c='red')
    plt.show()
