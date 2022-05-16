import math

import numpy as np

from environment.agents.deterministic_agent import DeterministicAgent
from environment.agents.stochastic_agent import StochasticAgent
from environment.robots.basic_robot import BasicRobot
from environment.robots.timing_robot import TimingRobot
from environment.stochastic_environment import StochasticEnvironment
from planners.stochastic_movement.stochastic_static_lack_planner import StochasticStaticLackPlanner
from utils.flow_utils import static_lack_moves
from utils.functions import meeting_height, line_trpv, map_into_2_pows, show_grid
from utils.point import Point


def test_direction():
    print('testing direction..')
    p1 = Point(1, 4)
    p2 = Point(1, 2)
    p3 = Point(-5, 2)

    assert p1.direction_with(p2) == 3 * math.pi / 2
    assert p2.direction_with(p3) == math.pi
    assert 3 * math.pi / 2 > p1.direction_with(p3) > math.pi


def test_shifted():
    print('testing shifted..')
    p1 = Point(4, 4)
    p2 = Point(4, 2)
    assert p1.shifted(distance=1, bearing=p1.direction_with(p2)) == Point(4, 3)
    assert p2.shifted(distance=3, bearing=math.pi) == p2.cartesian_shifted(x=-3, y=0)


def test_distance():
    print('testing distance..')
    p1 = Point(5, 5)
    p2 = Point(-5, -5)
    p3 = Point(7, 5)
    assert p3.distance_to(p1) == 2
    assert p1.distance_to(p2) == math.sqrt(200)


def test_meeting_height():
    print('testing meeting height..')
    robot = BasicRobot(loc=Point(2, 1), fv=2, d=1, is_disabling=True)
    agent1 = DeterministicAgent(Point(0, 0), 1)
    assert meeting_height(robot, agent1) == 1

    agent2 = DeterministicAgent(loc=Point(2, 2), v=1)
    assert meeting_height(robot, agent2) == 3

    agent3 = DeterministicAgent(Point(4, 0), 1)
    assert meeting_height(robot, agent1) == meeting_height(robot, agent3)


def test_map_into_2_pows():
    print('testing 2 pows mapping..')
    costs = [[5, 3, 1],
             [2, 4, 6]]
    modified_costs = map_into_2_pows(costs)
    assert modified_costs == [[2.0, 0.5, 0.125], [0.25, 1.0, 4.0]]

    costs = [[5, 3, 1, 15],
             [2, 4, 6, -4],
             [-2, 20, 0, -1]]
    modified_costs = map_into_2_pows(costs)
    assert modified_costs == [[4.0, 1.0, 0.25, 16.0], [0.5, 2.0, 8.0, 0.015625], [0.03125, 32.0, 0.125, 0.0625]]

    costs = [list(range(20)) for _ in range(20)]
    modified_costs = map_into_2_pows(costs)
    assert modified_costs[0] != 0


def test_line_trpv():
    print('testing line trpv..')
    agents = [DeterministicAgent(loc=Point(-1, 4), v=0),
              DeterministicAgent(loc=Point(-2, 3), v=0),
              DeterministicAgent(loc=Point(-4, -1), v=0),
              DeterministicAgent(loc=Point(-5, -2), v=0)]

    h = 0
    fv = 1
    makespan = 42

    trpv_data = line_trpv(h, fv, agents, makespan)
    movement, damage, t = trpv_data['ys'], trpv_data['damage'], trpv_data['t']
    assert movement == [-1, -2, 3, 4]
    assert damage == 1 * 4 + 1 * 3 + 5 * 2 + 1 * 1
    assert t == 2 + 2 + 4

    agents = [DeterministicAgent(loc=Point(-1, 100), v=0),
              DeterministicAgent(loc=Point(-2, 2), v=0),
              DeterministicAgent(loc=Point(-4, -1), v=0),
              DeterministicAgent(loc=Point(-5, -100), v=0)]

    h = 0
    fv = 1
    makespan = 42

    trpv_data = line_trpv(h, fv, agents, makespan)
    movement, damage, t = trpv_data['ys'], trpv_data['damage'], trpv_data['t']
    assert movement == [-1, 2, 100, -100]
    assert damage == 1 * 4 + 3 * 3 + 98 * 2 + 200 * 1
    assert t == 1 + 3 + 98 + 200

    agents = [DeterministicAgent(loc=Point(-1, -2), v=0),
              DeterministicAgent(loc=Point(-2, -2.1), v=0),
              DeterministicAgent(loc=Point(-3, -2.2), v=0),
              DeterministicAgent(loc=Point(-4, -2.3), v=0),
              DeterministicAgent(loc=Point(-5, 2), v=0),
              DeterministicAgent(loc=Point(8, -1000), v=0),
              DeterministicAgent(loc=Point(9, 100000), v=0)]

    h = 0.1
    fv = 1
    makespan = 42

    trpv_data = line_trpv(h, fv, agents, makespan)
    movement, damage, t = trpv_data['ys'], trpv_data['damage'], trpv_data['t']
    assert movement == [-2, -2.1, -2.2, -2.3, 2, -1000, 100000]
    assert damage == 2.1 * 7 + 0.1 * 6 + 0.1 * 5 + 0.1 * 4 + 3 * 4.3 + 1002 * 2 + 101000 * 1
    assert t == 2.4 + 4.3 + 1002 + 101000

    agents = [DeterministicAgent(loc=Point(-2, -5), v=1),
              DeterministicAgent(loc=Point(3, 4), v=1),
              DeterministicAgent(loc=Point(-10, 5), v=1),
              DeterministicAgent(loc=Point(-10, 6), v=1)]

    h = -1
    fv = 2
    makespan = 1

    movement = line_trpv(h, fv, agents, makespan)['ys']
    assert len(movement) == len(agents)

    agents = [DeterministicAgent(loc=Point(-2, 1), v=1),
              DeterministicAgent(loc=Point(3, 2), v=1),
              DeterministicAgent(loc=Point(-10, 3), v=1),
              DeterministicAgent(loc=Point(-10, 4), v=1)]

    h = 0
    fv = 2
    makespan = 0

    trpv_data = line_trpv(h, fv, agents, makespan)
    movement, damage, t = trpv_data['ys'], trpv_data['damage'], trpv_data['t']
    assert movement == [2, 4, 6, 8]
    assert damage == 4 + 3 + 2 + 1
    assert t == 4


def test_flow_moves():
    print('testing flow moves..')
    agents = [DeterministicAgent(loc=Point(-1, 9.6), v=1),
              DeterministicAgent(loc=Point(-6, 2), v=1),
              DeterministicAgent(loc=Point(10, 2), v=1),
              DeterministicAgent(loc=Point(8, -1), v=1)]

    robots = [BasicRobot(Point(-1, 9), 2, 1), BasicRobot(Point(3, 8), 2, 1)]

    fm = static_lack_moves(robots, agents, 10)
    movement, disabled = fm['movement'], fm['disabled']

    assert movement[robots[0]] == [Point(-6, 10)]
    assert movement[robots[1]] == [Point(10, 10), Point(8, 10)]
    assert set(disabled) == set(agents[1:])


def test_P_U_generation():
    b, br, bl = 10, 10, 0
    advance_distribution = (0.2, 0.6, 0.2)

    agents = [StochasticAgent(Point(3, 2), 1, advance_distribution),
              StochasticAgent(Point(5, 2), 1, advance_distribution),
              StochasticAgent(Point(6, 8), 1, advance_distribution),
              StochasticAgent(Point(8, 4), 1, advance_distribution)]

    robots = [BasicRobot(Point(1, 1), 2, 1),
              BasicRobot(Point(2, 0), 2, 1),
              BasicRobot(Point(4, 1), 2, 1)]

    environment = StochasticEnvironment(agents=agents, robots=robots, top_border=b, left_border=bl, right_border=br)

    Pa = environment.get_Pa(agents[0])
    Ua = environment.get_Ua(agents[0])
    PA = environment.PA
    UA = environment.UA

    assert (Ua >= Pa).all()
    assert (UA >= PA).all()
    assert (PA >= Pa).all()
    assert (UA >= Ua).all()

    delta = 1
    a = abs(PA - np.sum([environment.get_Pa(a) for a in agents]))
    assert (abs(PA - np.sum([environment.get_Pa(a) for a in agents])) < delta).all()
    assert (abs(UA - np.sum([environment.get_Ua(a) for a in agents])) < delta).all()


# show_grid(Pa[1], f'Pa of {agents[0]} at time {1}')
    # show_grid(Ua[1], f'Ua of {agents[0]} at time {1}')
    # show_grid(PA[1], f'PA matrix at time {1}')
    # show_grid(UA[1], f'UA matrix at time {1}')


def test_stochastic_lack_moves():
    agents = [StochasticAgent(loc=Point(2, 4), v=1, advance_distribution=[0, 1, 0]),
              StochasticAgent(loc=Point(5, 3), v=1, advance_distribution=[0, 1, 0]),
              StochasticAgent(loc=Point(7, 4), v=1, advance_distribution=[0, 1, 0])]

    robots = [TimingRobot(Point(2, 0), fv=2),
              TimingRobot(Point(5, 0), fv=2),
              TimingRobot(Point(7, 0), fv=2)]

    env = StochasticEnvironment(agents=agents, robots=robots, top_border=20,
                                right_border=10, left_border=0)

    planner = StochasticStaticLackPlanner()
    movement, time, damage, disabled, timing = planner.plan(env)


if __name__ == '__main__':
    test_direction()
    test_shifted()
    test_distance()
    test_meeting_height()
    test_line_trpv()
    test_map_into_2_pows()
    test_flow_moves()
    test_P_U_generation()
    test_stochastic_lack_moves()