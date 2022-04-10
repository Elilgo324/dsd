import random
from typing import Tuple

from environment.agents.base_agent import BaseAgent
from utils.point import Point


class StochasticAgent(BaseAgent):
    def __init__(self, loc: Point, v: float, advance_distribution: Tuple[float, float, float]):
        super().__init__(loc, v)
        self._advance_distribution = advance_distribution

    @property
    def advance_distribution(self) -> Tuple[float, float, float]:
        return self._advance_distribution

    def advance(self) -> None:
        # draw deviation according distribution
        deviation = random.choices([-1, 0, 1], weights=self._advance_distribution, k=1)[0]
        self.loc = Point(self.x + deviation, self.y + self.v)
