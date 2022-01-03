import math

from agents.fixed_velocity_agent import FixedVelocityAgent
from robots.basic_robot import BasicRobot
from utils.functions import meeting_height, line_trpv, map_into_2_pows
from utils.point import Point


def test_direction():
    p1 = Point(1, 4)
    p2 = Point(1, 2)
    p3 = Point(-5, 2)

    assert p1.direction_with(p2) == 3 * math.pi / 2
    assert p2.direction_with(p3) == math.pi
    assert 3 * math.pi / 2 > p1.direction_with(p3) > math.pi


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
    robot = BasicRobot(loc=Point(2, 1), fv=2, r=1, has_mode=True)
    agent1 = FixedVelocityAgent(Point(0, 0), 1)
    assert meeting_height(robot, agent1) == 1

    agent2 = FixedVelocityAgent(loc=Point(2, 2), v=1)
    assert meeting_height(robot, agent2) == 3

    agent3 = FixedVelocityAgent(Point(4, 0), 1)
    assert meeting_height(robot, agent1) == meeting_height(robot, agent3)


def test_map_into_2_pows():
    costs = [[5, 3, 1],
             [2, 4, 6]]
    modified_costs = map_into_2_pows(costs)
    # assert modified_costs == [[2.0, 0.5, 0.125], [0.25, 1.0, 4.0]]

    costs = [[5, 3, 1, 15],
             [2, 4, 6, -4],
             [-2, 20, 0, -1]]
    modified_costs = map_into_2_pows(costs)
    # assert modified_costs == [[4.0, 1.0, 0.25], [0.5, 2.0, 8.0], [0.0625, 16.0, 0.125]]

    costs = [list(range(200)) for _ in range(5)]
    modified_costs = map_into_2_pows(costs)
    a=4


def test_line_trpv():
    agents = [FixedVelocityAgent(loc=Point(-4, -3), v=1),
              FixedVelocityAgent(loc=Point(3, 2), v=1),
              FixedVelocityAgent(loc=Point(-10, -10), v=1),
              FixedVelocityAgent(loc=Point(-10, -9), v=1)]

    h = 1
    fv = 2
    makespan = 3

    movement = line_trpv(h, fv, agents, makespan)['ys']
    assert len(movement) == len(agents)

    agents = [FixedVelocityAgent(loc=Point(-2, -5), v=1),
              FixedVelocityAgent(loc=Point(3, 4), v=1),
              FixedVelocityAgent(loc=Point(-10, 5), v=1),
              FixedVelocityAgent(loc=Point(-10, 6), v=1)]

    h = -1
    fv = 2
    makespan = 7

    movement = line_trpv(h, fv, agents, makespan)['ys']
    assert len(movement) == len(agents)

    agents = [FixedVelocityAgent(loc=Point(-2, 1), v=1),
              FixedVelocityAgent(loc=Point(3, 2), v=1),
              FixedVelocityAgent(loc=Point(-10, 3), v=1)]

    h = -1
    fv = 2
    makespan = 1

    movement = line_trpv(h, fv, agents, makespan)['ys']
    assert len(movement) == len(agents)


if __name__ == '__main__':
    test_direction()
    test_shifted()
    test_distance()
    test_meeting_height()
    test_line_trpv()
    test_map_into_2_pows()
