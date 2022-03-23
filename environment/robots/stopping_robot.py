from environment.robots.basic_robot import BasicRobot
from utils.point import Point


class StoppingRobot(BasicRobot):
    def __init__(self, loc: Point, fv: float, r: float, has_mode: bool = True):
        super().__init__(loc, fv, r, has_mode)
        self._stop = False
        self._is_first_movement = True

    def set_stop(self, stop: float):
        self._stop = stop

    def advance(self) -> None:
        if not self._stop:
            if self._movement:
                target = self._movement[0]
                direction = self.loc.direction_with(target)

                if self._loc.distance_to(target) > self.fv:
                    self._loc = self.loc.shifted(distance=self.fv, bearing=direction)
                    if self._is_first_movement:
                        self._is_disabling = False
                else:
                    self.set_stop(True)
                    self._loc = self._movement[0]
                    self._movement.pop(0)
                    self._is_disabling = True
                    self._is_first_movement = False
