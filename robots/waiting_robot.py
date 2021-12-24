from robots.basic_robot import BasicRobot
from utils.point import Point


class WaitingRobot(BasicRobot):
    def __init__(self, loc: Point, fv: float, r: float):
        super().__init__(loc, fv, r)
        self._wait_time = 0
        self._is_first_movement = True

    def set_wait_time(self, wait_time: float):
        self._wait_time = wait_time

    def advance(self) -> None:
        if self._is_first_movement or self._wait_time <= 0:
            remain_dist = self.fv

            while self._movement:
                target = self._movement[0]
                direction = self.loc.direction_with(target)

                if self._loc.distance_to(target) > remain_dist:
                    self._loc = self.loc.shifted(distance=remain_dist, bearing=direction)
                    break
                else:
                    remain_dist -= self._loc.distance_to(self._movement[0])
                    self._loc = self._movement[0]
                    self._movement.pop(0)
                    self._is_first_movement = False

        self._wait_time -= 1
