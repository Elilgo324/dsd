import random
from typing import Tuple

from world.agents.base_agent import BaseAgent
from utils.point import Point


class StochasticAgent(BaseAgent):
    def __init__(self, loc: Point, v: float, advance_distribution: Tuple[float, float, float], left_border: float,
                 right_border: float):
        super().__init__(loc, v)
        self._advance_distribution = advance_distribution
        self._left_border = left_border
        self._right_border = right_border

    @property
    def advance_distribution(self) -> Tuple[float, float, float]:
        return self._advance_distribution

    def advance(self) -> None:
        # draw deviation according distribution
        deviation = random.choices([-1, 0, 1], weights=self._advance_distribution, k=1)[0]

        # if crosses border, advance forward
        if self.x + deviation < self._left_border or self.x + deviation >= self._right_border:
            deviation = 0

        self.loc = Point(self.x + deviation, self.y + self.v)
