from copy import deepcopy
from typing import List

from utils.consts import Consts
from utils.point import Point


class BasicRobot:
    def __init__(self, loc: Point, fv: float = Consts.ROBOT_DEF_SPEED, r: float = Consts.DISABLEMENT_RANGE):
        self._loc = loc
        self._fv = fv
        self._r = r
        self._movement = []

    @property
    def loc(self) -> Point:
        return self._loc

    @property
    def x(self) -> float:
        return self.loc.x

    @property
    def y(self) -> float:
        return self.loc.y

    @property
    def fv(self) -> float:
        return self._fv

    @property
    def r(self) -> float:
        return self._r

    def set_movement(self, movement: List[Point]) -> None:
        self._movement = movement

    def advance(self) -> None:
        if self._movement:
            prev_loc = self.loc
            target = self._movement[0]
            direction = self.loc.direction_with(target)

            if self._loc.distance_to(target) > self.fv:
                self._loc = self.loc.shifted(distance=self.fv, bearing=direction)
            else:
                self._loc = self._movement[0]
                self._movement.pop(0)

            if Consts.DEBUG:
                print(f'{str(self)}: {prev_loc} -> {self.loc}')

    def clone(self) -> 'BasicRobot':
        return deepcopy(self)

    def __str__(self):
        return f'Robot({self.x},{self.y})'
