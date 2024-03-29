from copy import deepcopy
from typing import List, Tuple

from utils.point import Point


class BasicRobot:
    def __init__(self, loc: Point, fv: float = 2, d: float = 2, is_disabling: bool = False):
        self._loc = loc
        self._fv = fv
        self._d = d
        self._movement = []
        self._is_disabling = is_disabling

    @property
    def loc(self) -> Point:
        return self._loc

    @loc.setter
    def loc(self, value):
        self._loc = value

    @property
    def x(self) -> float:
        return self.loc.x

    @property
    def y(self) -> float:
        return self.loc.y

    @property
    def xy(self) -> Tuple[float, float]:
        return self.loc.x, self.loc.y

    @property
    def fv(self) -> float:
        return self._fv

    @property
    def d(self) -> float:
        return self._d

    @property
    def is_disabling(self) -> bool:
        return self._is_disabling

    def set_movement(self, movement: List[Point]) -> None:
        self._movement = movement

    def advance(self) -> None:
        if len(self._movement) == 0:
            return

        target = self._movement[0]
        direction = self.loc.direction_with(target)

        if self._loc.distance_to(target) > self.fv:
            self._loc = self.loc.shifted(distance=self.fv, bearing=direction)
            self._is_disabling = False

        else:
            self._loc = self._movement.pop(0)
            self._is_disabling = True

    def clone(self) -> 'BasicRobot':
        return deepcopy(self)

    def __str__(self):
        return f'Robot({self.x},{self.y})'
