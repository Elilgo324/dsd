from abc import ABC
from copy import deepcopy

from utils.point import Point
from utils.consts import Consts


class BaseAgent(ABC):
    def __init__(self, loc: Point, v=Consts.AGENT_DEF_SPEED):
        self._loc = loc
        self._v = v
        self._is_disabled = False

    @property
    def x(self) -> float:
        return self._loc.x

    @property
    def y(self) -> float:
        return self._loc.y

    @property
    def loc(self):
        return self._loc

    @loc.setter
    def loc(self, value):
        self._loc = value

    @property
    def v(self):
        return self._v

    def clone(self):
        return deepcopy(self)

    def advance(self):
        pass

    def __str__(self):
        return f'Agent({round(self.x, 2)},{round(self.y, 2)})'
