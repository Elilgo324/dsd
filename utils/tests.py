import json
import math
from random import seed

import numpy as np

from planners.deterministic.baseline.iterative_assignment_planner import IterativeAssignmentPlanner
from planners.deterministic.partial_blockage.static_line_lack_planner import StaticLineLackPlanner
from planners.stochastic.baseline.stochastic_iterative_planner import StochasticIterativePlanner
from planners.stochastic.partial_blockage.stochastic_static_lack_planner import StochasticStaticLackPlanner
from world.agents.deterministic_agent import DeterministicAgent
from world.agents.stochastic_agent import StochasticAgent
from world.robots.basic_robot import BasicRobot
from world.robots.timing_robot import TimingRobot
from world.stochastic_environment import StochasticEnvironment
from utils.consts import Consts
from utils.algorithms import static_lack_moves, line_trpv, iterative_assignment
from utils.functions import meeting_height, map_into_2_pows, integrate_gauss, sigma_t, meeting_points_with_sigmas, \
    sample_point
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


def test_gauss():
    print('testing gauss..')
    mu = 1
    sigma1 = 1
    sigma2 = 2
    assert integrate_gauss(mu, sigma1, -10, 10) == 1
    assert integrate_gauss(mu, sigma1, -2, 2) > integrate_gauss(mu, sigma2, -2, 2)
    assert integrate_gauss(mu, sigma1, mu - 2 * sigma1, mu + 2 * sigma1) > 0.95 * integrate_gauss(mu, sigma1, -1000,
                                                                                                  1000)
    assert integrate_gauss(mu, sigma1, mu - sigma1, mu + sigma1) > 0.68 * integrate_gauss(mu, sigma2, -1000, 1000)


def test_future_sigmas():
    print('testing future sigmas..')
    sigma = 4
    var = sigma ** 2
    dt = 8
    assert abs(sigma_t(sigma, dt) ** 2 - var * dt) < Consts.EPSILON


def test_meeting_height_sigmas():
    print('testing meeting height sigmas..')
    mu = 7
    sigma = 1.5
    border = 50
    agent = StochasticAgent(loc=Point(mu, 8), v=1, sigma=sigma)
    robot = BasicRobot(loc=Point(4, 10), fv=2)
    left_point, right_point = meeting_points_with_sigmas(robot, agent, border, res=0.01)

    assert abs(left_point.distance_to(robot.loc) / robot.fv - left_point.y + agent.y) < Consts.EPSILON
    assert abs(right_point.distance_to(robot.loc) / robot.fv - right_point.y + agent.y) < Consts.EPSILON

    mu = 30
    sigma = 4
    border = 500
    agent = StochasticAgent(loc=Point(mu, 0), v=1, sigma=sigma)
    robot = BasicRobot(loc=Point(100, 10), fv=1.5)
    left_point, right_point = meeting_points_with_sigmas(robot, agent, border, res=0.01)

    assert abs(left_point.distance_to(robot.loc) / robot.fv - left_point.y + agent.y) < Consts.EPSILON
    assert abs(right_point.distance_to(robot.loc) / robot.fv - right_point.y + agent.y) < Consts.EPSILON


def test_iterative_assignment():
    print('testing iterative assignment..')
    agents = [DeterministicAgent(loc=Point(-3, 2), v=1),
              DeterministicAgent(loc=Point(-3, 14), v=1),
              DeterministicAgent(loc=Point(11, 5), v=1),
              DeterministicAgent(loc=Point(11, 101), v=1)]
    robots = [BasicRobot(loc=Point(-3,0), fv=2, d=2),
              BasicRobot(loc=Point(11,1), fv=2, d=2)]

    stats = iterative_assignment(robots, agents, 1000)
    movement = stats['movement']
    damage = stats['damage']
    num_disabled = stats['num_disabled']

    assert damage == (2 + 14 + 4 + 100) and num_disabled == 4
    assert movement == {robots[0]: [Point(robots[0].x, 4), Point(robots[0].x, 28)],
                        robots[1]: [Point(robots[1].x, 9), Point(robots[1].x, 201)]}


def test_iterative_stochastic_agrees_with_non_stochastic_zero_sigma():
    print('testing iterative stochastic agrees with non stochastic..')
    seed(42)

    agents = [StochasticAgent(loc=sample_point(10, 110, 10, 110, True), v=1, sigma=0) for _ in range(10)]
    robots = [BasicRobot(sample_point(0, 120, 0, 10, True), 2, 1) for _ in range(5)]

    env = StochasticEnvironment(agents=agents, robots=robots, top_border=310, right_border=110, left_border=10)

    det_planner = IterativeAssignmentPlanner()
    stoch_planner = StochasticIterativePlanner()

    det_results = det_planner.plan(env)
    stoch_results = stoch_planner.plan(env)
    assert det_results == stoch_results


def test_iterative_stochastic_less_than_non_stochastic():
    print('testing iterative stochastic less than non stochastic..')
    seed(42)

    agents = [StochasticAgent(loc=sample_point(10, 110, 10, 110, True), v=1, sigma=0.1) for _ in range(10)]
    robots = [BasicRobot(sample_point(0, 120, 0, 10, True), 2, 1) for _ in range(5)]

    env = StochasticEnvironment(agents=agents, robots=robots, top_border=310, right_border=110, left_border=10)

    det_planner = IterativeAssignmentPlanner()
    stoch_planner = StochasticIterativePlanner()

    _, _, damage, num_disabled = det_planner.plan(env)
    _, _, expected_damage, expected_num_disabled = stoch_planner.plan(env)
    assert damage < expected_damage and num_disabled > expected_num_disabled


if __name__ == '__main__':
    test_direction()
    test_shifted()
    test_distance()
    test_meeting_height()
    test_line_trpv()
    test_map_into_2_pows()
    test_flow_moves()
    test_gauss()
    test_future_sigmas()
    test_meeting_height_sigmas()
    test_iterative_assignment()
    test_iterative_stochastic_agrees_with_non_stochastic_zero_sigma()
    test_iterative_stochastic_less_than_non_stochastic()
