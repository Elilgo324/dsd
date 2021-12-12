from manim.utils.point import Point
from utils.consts import Consts
import math

def test_direction():
    print('*** testing direction ***')
    p1 = Point(1, 4)
    p2 = Point(1, 2)
    p3 = Point(-5, 2)

    print(p1.direction_with(p2) == 3 * math.pi / 2)
    print(p2.direction_with(p3) == math.pi)
    print(p1.direction_with(p3) < 3 * math.pi / 2 and p1.direction_with(p3) > math.pi)


def test_shifted():
    print('*** testing shifted ***')
    p1 = Point(4, 4)
    p2 = Point(4, 2)
    print(p1.shifted(distance=1,bearing=p1.direction_with(p2)) == Point(4,3))
    print(p2.shifted(distance=3,bearing=math.pi) == p2.cartesian_shifted(x=-3,y=0))

def test_distance():
    print('*** testing distance ***')
    p1 = Point(5, 5)
    p2 = Point(-5, -5)
    p3 = Point(7,5)
    print(p3.distance_to(p1) == 2)
    print(p1.distance_to(p2) == math.sqrt(200))


if __name__ == '__main__':
    test_direction()
    test_shifted()
    test_distance()