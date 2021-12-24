import math

from agents.fixed_velocity_agent import FixedVelocityAgent
from utils.point import Point
from robots.basic_robot import BasicRobot
from utils.functions import meeting_height


def test_direction():
    p1 = Point(1, 4)
    p2 = Point(1, 2)
    p3 = Point(-5, 2)

    assert p1.direction_with(p2) == 3 * math.pi / 2
    assert p2.direction_with(p3) == math.pi
    assert p1.direction_with(p3) < 3 * math.pi / 2 and p1.direction_with(p3) > math.pi


def test_shifted():
    p1 = Point(4, 4)
    p2 = Point(4, 2)
    assert p1.shifted(distance=1, bearing=p1.direction_with(p2)) == Point(4, 3)
    assert p2.shifted(distance=3, bearing=math.pi) == p2.cartesian_shifted(x=-3, y=0)


def test_distance():
    p1 = Point(5, 5)
    p2 = Point(-5, -5)
    p3 = Point(7, 5)
    assert p3.distance_to(p1) == 2
    assert p1.distance_to(p2) == math.sqrt(200)


def test_meeting_height():
    robot = BasicRobot(loc=Point(2,1),fv=2,r=1)
    agent1 = FixedVelocityAgent(Point(0,0),1)
    assert meeting_height(robot, agent1) == 1

    agent2 = FixedVelocityAgent(loc=Point(2,2), v=1)
    assert meeting_height(robot, agent2) == 3

    agent3 = FixedVelocityAgent(Point(4,0),1)
    assert meeting_height(robot,agent1) == meeting_height(robot,agent3)


if __name__ == '__main__':
    test_direction()
    test_shifted()
    test_distance()
    test_meeting_height()
