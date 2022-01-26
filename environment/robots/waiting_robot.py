from environment.robots.basic_robot import BasicRobot
from utils.point import Point


class WaitingRobot(BasicRobot):
    def __init__(self, loc: Point, fv: float, r: float, has_mode: bool):
        super().__init__(loc, fv, r, has_mode)
        self._wait_time = 0
        self._is_first_movement = True

    def set_wait_time(self, wait_time: float):
        self._wait_time = wait_time

    def advance(self) -> None:
        if self._is_first_movement or self._wait_time <= 0:
            if self._movement:
                target = self._movement[0]
                direction = self.loc.direction_with(target)

                if self._loc.distance_to(target) > self.fv:
                    self._loc = self.loc.shifted(distance=self.fv, bearing=direction)
                    self._is_disabling = False
                else:
                    self._loc = self._movement[0]
                    self._movement.pop(0)
                    self._is_disabling = True
                    self._is_first_movement = False

        self._wait_time -= 1
