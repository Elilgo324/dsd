import math
import operator
import os
from datetime import datetime
from math import sqrt, floor
from random import uniform, randint
from typing import List, Tuple, Dict, Union

import seaborn as sb
import matplotlib.pyplot as plt
import imageio
import networkx as nx
import numpy as np
from scipy.optimize import linear_sum_assignment

from environment.agents.base_agent import BaseAgent
from environment.environment import Environment
from environment.robots.basic_robot import BasicRobot
from utils.consts import Consts
from utils.point import Point


def sample_point(x_min: float, x_max: float, y_min: float, y_max: float, is_int: bool=False) -> Point:
    if is_int:
        return Point(randint(int(x_min), int(x_max)-1), randint(int(y_min), int(y_max)))
    return Point(uniform(x_min, x_max), uniform(y_min, y_max))


def meeting_height(robot: BasicRobot, agent: BaseAgent) -> float:
    f = robot.fv / agent.v
    a, b = robot.x, robot.y
    c, d = agent.x, agent.y

    if math.isclose(a, c) and math.isclose(b, d):
        return b

    inside_sqrt = a ** 2 * f ** 2 - a ** 2 - 2 * a * c * f ** 2 + 2 * a * c \
                  + b ** 2 * f ** 2 - 2 * b * d * f ** 2 + c ** 2 * f ** 2 - c ** 2 + d ** 2 * f ** 2
    num = sqrt(inside_sqrt) - b + d * f ** 2
    den = f ** 2 - 1
    h = num / den

    if h < agent.y:
        raise ValueError('meeting height cannot be lower than agent')

    return h


def plot_environment(robots: List[BasicRobot], agents: List[BaseAgent],
                     env: Environment, config: Dict[str, float]) -> None:
    plt.clf()
    plt.xlim(0, config['x_size'] + 2 * config['x_buffer'])
    plt.ylim(0, config['y_size'] + 2 * config['y_buffer'])
    plt.plot([config['x_buffer'], config['x_buffer'], config['x_buffer'] + config['x_size'], config['x_buffer'] +
              config['x_size'], config['x_buffer']],
             [config['y_buffer'], config['y_buffer'] + config['y_size'], config['y_buffer'] + config['y_size'],
              config['y_buffer'], config['y_buffer']], c='black')
    plt.scatter([r.x for r in robots], [r.y for r in robots], c='blue')
    # for i in range(len(robots)):
    #     plt.annotate(i, (robots[i].x, robots[i].y))
    plt.scatter([a.x for a in agents], [a.y for a in agents], c='red')
    plt.title(env.stats(), fontsize=10)
    # plt.gca().set_aspect('equal', adjustable='box')
    plt.savefig(f'./plots/{env.step}')


def create_gif_from_plots(prefix: str = ''):
    filenames = os.listdir('./plots/')
    filenames = [file[:-4] for file in filenames]
    with imageio.get_writer(f'./gifs/{prefix}-{datetime.now().minute}.gif', mode='I') as writer:
        for filename in sorted(filenames, key=int):
            image = imageio.imread('./plots/' + filename + '.png')
            writer.append_data(image)

    # remove files
    for filename in filenames:
        os.remove('./plots/' + filename + '.png')


def write_report(planner: str,
                 num_agents: int,
                 num_robots: int,
                 f: float,
                 d: float,
                 active_time: float,
                 planner_time: float,
                 damage: float,
                 num_disabled: int,
                 file_name: str = 'results.csv') -> None:
    stats = [planner, num_agents, num_robots, f, d, active_time, planner_time, damage, num_disabled]

    if not os.path.exists(file_name):
        file = open(file_name, 'a+')
        file.write('planner,num_agents,num_robots,f,d,active_time,planner_time,damage,'
                   'num_disabled\n')
    else:
        file = open(file_name, 'a+')

    file.write(",".join([str(s) for s in stats]))
    file.write('\n')


def refine_movement(movement):
    pass


def map_into_2_pows(costs: List[List[float]]) -> List[List[float]]:
    rows_num = len(costs)
    cols_num = len(costs[0])

    enumerate_object = enumerate([item for sublist in costs for item in sublist])
    sorted_pairs = sorted(enumerate_object, key=operator.itemgetter(1))
    sorted_indices = [index for index, element in sorted_pairs]

    pows = 2 ** -int((rows_num * cols_num) / 2)
    for i in sorted_indices:
        row = floor(i / cols_num)
        col = i % cols_num
        costs[row][col] = pows
        pows *= 2

    return costs


def line_trpv(h: float, fv: float, agents: List['BaseAgent'], makespan: float) \
        -> Dict[str, Union[float, Dict['BasicRobot', List[Point]]]]:
    v = agents[0].v

    def time_to_meet(source, target):
        if source < target:
            return abs(source - target) / (fv - v)
        return abs(source - target) / (fv + v)

    agents_ys = [a.y + v * makespan for a in agents]

    # X are the agents above h and Y are below
    X = sorted([a for a in agents_ys if a > h + Consts.EPSILON])
    Y = sorted([a for a in agents_ys if a < h - Consts.EPSILON], reverse=True)

    time_to_meet_h = {a: time_to_meet(h, a) for a in agents_ys}

    # trivial solution if no agents above or below
    if len(X) == 0 and len(Y) == 0:
        return {'damage': 0, 'ys': [], 't': 0}

    if len(X) == 0:
        return {'damage': sum([time_to_meet_h[a] for a in X + Y]),
                'ys': [a + v * time_to_meet_h[a] for a in X + Y],
                't': time_to_meet_h[Y[-1]]}

    if len(Y) == 0:
        return {'damage': sum([time_to_meet_h[a] for a in X + Y]),
                'ys': [a + v * time_to_meet_h[a] for a in X + Y],
                't': time_to_meet_h[X[-1]]}

    # tables for the state of (x,y) and (y,x) for each x and y
    XY = {x: {y: {'damage': 0, 'ys': [], 't': 0} for y in Y} for x in X}
    YX = {y: {x: {'damage': 0, 'ys': [], 't': 0} for x in X} for y in Y}

    # fill first row of XY
    for i in range(len(Y)):
        x1 = X[0]
        yi = Y[i]
        time_h_yi = time_to_meet(h, yi)
        time_yi_x1 = time_to_meet(yi, x1)
        XY[x1][yi]['damage'] = sum([time_to_meet_h[y] for y in Y[:i + 1]]) \
                               + (len(agents) - i - 1) * (time_h_yi + time_yi_x1)
        XY[x1][yi]['ys'] = [y + v * time_to_meet_h[y] for y in Y[:i + 1]] + [x1 + v * (time_h_yi + time_yi_x1)]
        XY[x1][yi]['t'] = time_h_yi + time_yi_x1

    # fill first row of YX
    for i in range(len(X)):
        y1 = Y[0]
        xi = X[i]
        time_h_xi = time_to_meet(h, xi)
        time_xi_y1 = time_to_meet(xi, y1)
        YX[y1][xi]['damage'] = sum([time_to_meet_h[x] for x in X[:i + 1]]) \
                               + (len(agents) - i - 1) * (time_h_xi + time_xi_y1)
        YX[y1][xi]['ys'] = [x + v * time_to_meet_h[x] for x in X[:i + 1]] + [y1 + v * (time_h_xi + time_xi_y1)]
        YX[y1][xi]['t'] = time_h_xi + time_xi_y1

    # fill in diagonals
    for i in range(len(X) + len(Y) - 2):
        # top right XY
        if i < len(Y):
            idx_x = 1
            idx_y = i
        else:
            idx_x = i + 2 - len(Y)
            idx_y = len(Y) - 1

        # fill diagonal XY
        while idx_x < len(X) and idx_y >= 0:
            num_living_agents = len(agents) - (idx_x + idx_y) - 1

            cur_x = X[idx_x]
            prev_x = X[idx_x - 1]
            cur_y = Y[idx_y]

            time_reaching_from_up = time_to_meet(prev_x, cur_x)
            time_reaching_from_down = time_to_meet(cur_y, cur_x)

            damage_reaching_from_up = XY[prev_x][cur_y]['damage'] \
                                      + num_living_agents * time_reaching_from_up
            damage_reaching_from_down = YX[cur_y][prev_x]['damage'] \
                                        + num_living_agents * time_reaching_from_down

            if damage_reaching_from_up < damage_reaching_from_down:
                XY[cur_x][cur_y]['damage'] = damage_reaching_from_up
                XY[cur_x][cur_y]['t'] = XY[prev_x][cur_y]['t'] + time_reaching_from_up
                path_reaching_from_up = XY[prev_x][cur_y]['ys']
                XY[cur_x][cur_y]['ys'] = path_reaching_from_up + [cur_x + v * XY[cur_x][cur_y]['t']]
            else:
                XY[cur_x][cur_y]['damage'] = damage_reaching_from_down
                XY[cur_x][cur_y]['t'] = YX[cur_y][prev_x]['t'] + time_reaching_from_down
                path_reaching_from_down = YX[cur_y][prev_x]['ys']
                XY[cur_x][cur_y]['ys'] = path_reaching_from_down + [cur_x + v * XY[cur_x][cur_y]['t']]

            idx_x += 1
            idx_y -= 1

        # top right YX
        if i < len(X):
            idx_y = 1
            idx_x = i
        else:
            idx_y = i + 2 - len(X)
            idx_x = len(X) - 1

        # fill diagonal YX
        while idx_y < len(Y) and idx_x >= 0:
            num_living_agents = len(agents) - (idx_x + idx_y) - 1

            cur_y = Y[idx_y]
            prev_y = Y[idx_y - 1]
            cur_x = X[idx_x]

            time_reaching_from_down = time_to_meet(prev_y, cur_y)
            time_reaching_from_up = time_to_meet(cur_x, cur_y)

            damage_reaching_from_down = YX[prev_y][cur_x]['damage'] \
                                        + num_living_agents * time_reaching_from_down
            damage_reaching_from_up = XY[cur_x][prev_y]['damage'] \
                                      + num_living_agents * time_reaching_from_up

            if damage_reaching_from_down < damage_reaching_from_up:
                YX[cur_y][cur_x]['damage'] = damage_reaching_from_down
                YX[cur_y][cur_x]['t'] = YX[prev_y][cur_x]['t'] + time_reaching_from_down
                path_reaching_from_down = YX[prev_y][cur_x]['ys']
                YX[cur_y][cur_x]['ys'] = path_reaching_from_down + [cur_y + v * YX[cur_y][cur_x]['t']]
            else:
                YX[cur_y][cur_x]['damage'] = damage_reaching_from_up
                YX[cur_y][cur_x]['t'] = XY[cur_x][prev_y]['t'] + time_reaching_from_up
                path_reaching_from_up = XY[cur_x][prev_y]['ys']
                YX[cur_y][cur_x]['ys'] = path_reaching_from_up + [cur_y + v * YX[cur_y][cur_x]['t']]

            idx_y += 1
            idx_x -= 1

    # compare the bottom right cells of the tables
    damage_up = XY[X[-1]][Y[-1]]['damage']
    damage_down = YX[Y[-1]][X[-1]]['damage']

    if damage_up < damage_down:
        return XY[X[-1]][Y[-1]]
    return YX[Y[-1]][X[-1]]


def iterative_assignment(robots: List['BasicRobot'], agents_copy: List['BaseAgent'], border: float) \
        -> Dict[str, Union[Dict, int, float]]:
    MY_INF = 1_000_000
    fv = robots[0].fv

    movement = {robot: [] for robot in robots}
    busy_time = {robot: 0 for robot in robots}
    avoided_damage = 0
    expected_num_disabled = 0
    potential_damage = sum([border - a.y for a in agents_copy])

    # assign while there are agents alive
    while len(agents_copy) > 0:
        # calculate assignment costs
        distances = [[] for _ in range(len(robots))]
        meeting_times = {robot: {agent: None for agent in agents_copy} for robot in robots}
        meeting_points = {robot: {agent: None for agent in agents_copy} for robot in robots}
        for i in range(len(robots)):
            # updated robot
            robot_at_time = robots[i].clone()
            if len(movement[robots[i]]) > 0:
                robot_at_time.loc = movement[robots[i]][-1]

            for a in agents_copy:
                # updated agent
                agent_at_time = BaseAgent(Point(a.x, a.y + busy_time[robots[i]] * a.v), a.v)

                x_meeting = a.x
                y_meeting = meeting_height(robot_at_time, agent_at_time)

                # if meeting outside the border, cost is inf
                if y_meeting > border:
                    distances[i].append(MY_INF)
                    meeting_points[robots[i]][a] = None
                else:
                    meeting_point = Point(x_meeting, y_meeting)
                    dist = robot_at_time.loc.distance_to(meeting_point)
                    distances[i].append(dist)
                    meeting_times[robots[i]][a] = dist / fv
                    meeting_points[robots[i]][a] = meeting_point

        # apply optimal assignment
        optimal_assignment = linear_sum_assignment(distances)
        assigned_robots = optimal_assignment[0]
        assigned_agents = optimal_assignment[1]

        # update according optimal assignment
        agents_to_remove = []
        non_assigned_num = 0
        for i in range(len(assigned_robots)):
            assigned_robot = robots[assigned_robots[i]]
            assigned_agent = agents_copy[assigned_agents[i]]
            meeting_point = meeting_points[assigned_robot][assigned_agent]

            # if meeting outside the border, continue
            if meeting_point is None:
                non_assigned_num += 1
                continue

            # else, update values
            movement[assigned_robot].append(meeting_point)
            busy_time[assigned_robot] += meeting_times[assigned_robot][assigned_agent]

            avoided_damage += (border - meeting_point.y)
            expected_num_disabled += 1
            agents_to_remove.append(assigned_agent)

        if non_assigned_num == len(assigned_agents):
            break

        for a in agents_to_remove:
            agents_copy.remove(a)

    expected_damage = potential_damage - avoided_damage
    completion_time = max(busy_time.values())
    return {'movement': movement,
            'completion_time': completion_time,
            'damage': expected_damage,
            'num_disabled': expected_num_disabled}

def show_grid(M, title_str = None):
    min_val = np.min(M)
    max_val = np.max(M)
    sum_val = np.sum(M)

    M = np.flipud(M)
    fig, ax = plt.subplots(figsize=(20, 6))

    if title_str is not None:
        plt.title(title_str)

    sb.heatmap(M, annot=True, fmt=".2f", cmap='Blues', vmin=np.min(M), vmax=np.max(M), cbar_kws={"shrink": .8})
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

    print(f'sum of mat cells: {sum_val}')
    print(f'max value is: {max_val}')
    print(f'min value is: {min_val}')
