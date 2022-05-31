import random
from typing import Tuple

from world.agents.base_agent import BaseAgent
from utils.point import Point
from numpy import random


class StochasticAgent(BaseAgent):
    def __init__(self, loc: Point, v: float, sigma: float):
        super().__init__(loc, v)
        self._sigma = sigma

    @property
    def sigma(self) -> float:
        return self._sigma

    def advance(self) -> None:
        self.loc = Point(random.normal(loc=self.x, scale=self.sigma), self.y + self.v)
