
from typing import List

from environment.robots.basic_robot import BasicRobot
from utils.point import Point


class TimingRobot(BasicRobot):
    def __init__(self, loc: Point, fv: float = 2, d: float = 2, is_disabling: bool = False):
        super().__init__(loc, fv, d, is_disabling)
        self._timing = []
        self._timer = 0

    def set_timing(self, timing: List[float]) -> None:
        self._timing = timing

        if len(self._timing) != len(self._movement):
            print(f'{len(self._timing)}!={len(self._movement)}')

    def advance(self) -> None:
        self._timer += 1

        if len(self._movement) == 0:
            return

        if self._timer >= self._timing[0]:
            self._timing.pop(0)
            self._movement.pop(0)
            if len(self._movement) == 0:
                return

            self._loc = self._movement[0]
            self._is_disabling = True
        else:
            direction = self.loc.direction_with(self._movement[0])
            self._loc = self.loc.shifted(distance=self.fv, bearing=direction)
            self._is_disabling = False


