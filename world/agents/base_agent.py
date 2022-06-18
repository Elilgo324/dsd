from abc import ABC
from copy import deepcopy
from typing import Tuple

from utils.point import Point


class BaseAgent(ABC):
    def __init__(self, loc: Point, v: float = 1):
        self._loc = loc
        self._v = v
        self._is_disabled = False

    @property
    def x(self) -> float:
        return self._loc.x

    @x.setter
    def x(self, value) -> None:
        self._loc.x = value

    @property
    def y(self) -> float:
        return self._loc.y

    @property
    def xy(self) -> Tuple[float, float]:
        return self._loc.x, self._loc.y

    @property
    def loc(self) -> Point:
        return self._loc

    @loc.setter
    def loc(self, value) -> None:
        self._loc = value

    @property
    def v(self) -> float:
        return self._v

    @v.setter
    def v(self, value) -> None:
        self._v = value

    def clone(self) -> 'BaseAgent':
        return deepcopy(self)

    def advance(self) -> None:
        pass

    def __str__(self) -> str:
        return f'Agent({round(self.x, 2)},{round(self.y, 2)})'
