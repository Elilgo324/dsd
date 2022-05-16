import math

from utils.consts import Consts


class Point:
    def __init__(self, x: float, y: float):
        self._x = x
        self._y = y

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @x.setter
    def x(self, val):
        self._x = val

    @y.setter
    def y(self, val):
        self._y = val

    def __eq__(self, other):
        return self.x - other.x < Consts.EPSILON and self.y - other.y < Consts.EPSILON

    def __str__(self):
        return f'({self.x},{self.y})'

    def __hash__(self):
        return hash((self.x, self.y))

    def distance_to(self, other):
        delta_x = self.x - other.x
        delta_y = self.y - other.y
        return math.sqrt(delta_x ** 2 + delta_y ** 2)

    def cartesian_shifted(self, x, y):
        return Point(self.x + x, self.y + y)

    def shifted(self, distance, bearing):
        x = distance * math.cos(bearing)
        y = distance * math.sin(bearing)
        return self.cartesian_shifted(x, y)

    def direction_with(self, other):
        delta_x = other.x - self.x
        delta_y = other.y - self.y
        theta = math.atan2(delta_y, delta_x)
        if theta < 0:
            theta += 2 * math.pi
        return theta
