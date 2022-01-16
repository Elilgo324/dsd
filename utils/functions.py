import math
import operator
import os
from datetime import datetime
from math import sqrt, floor
from random import uniform
from typing import List, Tuple, Dict, Union

# import matplotlib.pyplot as plt
# import imageio
import networkx as nx
from munkres import Munkres

from agents.base_agent import BaseAgent
from agents.fixed_velocity_agent import FixedVelocityAgent
from environment import Environment
from robots.basic_robot import BasicRobot
from utils.consts import Consts
from utils.point import Point


def sample_point(x_min: float, x_max: float, y_min: float, y_max: float) -> Point:
    return Point(uniform(x_min, x_max), uniform(y_min, y_max))


def meeting_height(robot: BasicRobot, agent: BaseAgent) -> float:
    f = robot.fv / agent.v
    a, b = robot.x, robot.y
    c, d = agent.x, agent.y
    inside_sqrt = a ** 2 * f ** 2 - a ** 2 - 2 * a * c * f ** 2 + 2 * a * c \
                  + b ** 2 * f ** 2 - 2 * b * d * f ** 2 + c ** 2 * f ** 2 - c ** 2 + d ** 2 * f ** 2
    num = sqrt(inside_sqrt) - b + d * f ** 2
    den = f ** 2 - 1
    h = num / den

    if h < agent.y:
        raise ValueError('meeting height cannot be lower than agent')

    return h


def plot_environment(robots: List[BasicRobot], agents: List[BaseAgent],
                     env: Environment, config: Dict[str,float]) -> None:
    plt.clf()
    plt.xlim(0, config['x_size'] + 2 * config['x_buffer'])
    plt.ylim(0, config['y_size'] + 2 * config['y_buffer'])
    plt.plot([config['x_buffer'], config['x_buffer'], config['x_buffer'] + config['x_size'], config['x_buffer'] +
              config['x_size'], config['x_buffer']],
             [config['y_buffer'], config['y_buffer'] + config['y_size'], config['y_buffer'] + config['y_size'],
              config['y_buffer'], config['y_buffer']], c='black')
    plt.scatter([r.x for r in robots], [r.y for r in robots], c='blue')
    for i in range(len(robots)):
        plt.annotate(i, (robots[i].x, robots[i].y))
    plt.scatter([a.x for a in agents], [a.y for a in agents], c='red')
    plt.title(env.stats(), fontsize=10)
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
                 completion_time: float,
                 planner_time: float,
                 damage: float,
                 num_disabled: int,
                 file_name: str = 'results.csv') -> None:
    stats = [planner, num_agents, num_robots, f, d, active_time, completion_time, planner_time, damage, num_disabled]

    if not os.path.exists(file_name):
        file = open(file_name, 'a+')
        file.write('planner,num_agents,num_robots,f,d,active_time,completion_time,planner_time,damage,'
                   'num_disabled\n')
    else:
        file = open(file_name, 'a+')

    file.write(",".join([str(s) for s in stats]))
    file.write('\n')


def refine_movement(movement):
    for i in range(len(movement)):
        if 0 < i < len(movement) - 1 and step.y[i - 1] < step.y[i] < step.y[j + 1]:
            continue
    return movement


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

    # compare the bottow right cells of the tables
    damage_up = XY[X[-1]][Y[-1]]['damage']
    damage_down = YX[Y[-1]][X[-1]]['damage']

    if damage_up < damage_down:
        return XY[X[-1]][Y[-1]]
    return YX[Y[-1]][X[-1]]


def iterative_assignment(robots: List['BasicRobot'], agents_copy: List['BaseAgent']) \
        -> Dict[str, Union[Dict, int, float]]:
    movement = {robot: [] for robot in robots}
    free_time = {robot: 0 for robot in robots}
    expected_damage = 0
    expected_num_disabled = 0

    while len(agents_copy) > 0:
        distances = [[] for _ in range(len(robots))]
        for i in range(len(robots)):
            for a in agents_copy:
                robot_at_time = robots[i].clone()
                if len(movement[robots[i]]) > 0:
                    robot_at_time.loc = movement[robots[i]][-1]

                x_meeting = a.x
                agent_at_time = BaseAgent(Point(a.x, a.y + free_time[robots[i]] * a.v), a.v)
                y_meeting = meeting_height(robot_at_time, agent_at_time)
                distances[i].append(robot_at_time.loc.distance_to(Point(x_meeting, y_meeting)))

        optimal_assignment = Munkres().compute(distances)
        assigned_robots, assigned_agents = map(list, zip(*optimal_assignment))

        for i in range(len(assigned_robots)):
            assigned_robot = robots[assigned_robots[i]]
            assigned_agent = agents_copy[assigned_agents[i]]

            expected_damage += free_time[assigned_robot]
            expected_num_disabled += 1

            prev_loc = assigned_robot.loc
            if len(movement[assigned_robot]) > 0:
                prev_loc = movement[assigned_robot][-1]

            x_meeting = assigned_agent.x
            y_meeting = meeting_height(assigned_robot, assigned_agent)
            meeting_point = Point(x_meeting, y_meeting)

            movement[assigned_robot].append(meeting_point)
            free_time[assigned_robot] += prev_loc.distance_to(meeting_point) / assigned_robot.fv

        agents_to_remove = [agents_copy[i] for i in assigned_agents]
        for a in agents_to_remove:
            agents_copy.remove(a)

    active_time = completion_time = max(free_time.values())
    return {'movement': movement,
            'active_time': active_time,
            'completion_time': completion_time,
            'damage': expected_damage,
            'num_disabled': expected_num_disabled}


def flow_moves(robots: List['BasicRobot'], agents: List['BaseAgent'], h: float):
    v = agents[0].v
    fv = robots[0].fv

    def can_stop_on_line(r: Tuple[float, float], a: Tuple[float, float], h: float):
        x_a, y_a = a
        x_r, y_r = r
        t_a = (h - y_a) / v
        t_r = math.sqrt((x_a - x_r) ** 2 + (h - y_r) ** 2) / fv
        return t_r <= t_a

    g = nx.DiGraph()

    # create robots
    for robot in robots:
        g.add_node(str(robot), pos=robot.xy, color='blue')

    # create agents divided to in and out
    for agent in agents:
        g.add_node(str(agent) + '_i', pos=(agent.x - 0.5, agent.y), color='red')
        g.add_node(str(agent) + '_o', pos=(agent.x + 0.5, agent.y), color='red')
        g.add_edge(str(agent) + '_i', str(agent) + '_o', weight=-1, capacity=1)

    # add edges from robots to agents
    for robot in robots:
        for agent in agents:
            if can_stop_on_line(r=robot.xy, a=agent.xy, h=h):
                g.add_edge(str(robot), str(agent) + '_i', weight=0, capacity=1)

    # add edges between agents
    for agent1 in agents:
        for agent2 in agents:
            if agent1 is agent2:
                continue
            if can_stop_on_line(r=(agent1.x, h), a=(agent2.x, h - (agent1.y - agent2.y)), h=h):
                g.add_edge(str(agent1) + '_o', str(agent2) + '_i', weight=0, capacity=1)

    # add dummy source and target to use flow
    for robot in robots:
        g.add_edge('s', str(robot), weight=0, capacity=1)
        g.add_edge(str(robot), 't', weight=0, capacity=1)

    for agent in agents:
        g.add_edge(str(agent) + '_o', 't', weight=0, capacity=1)

    flow = nx.max_flow_min_cost(g, 's', 't')

    # delete all edges without flow
    edges_to_delete = []
    for key1, val1 in flow.items():
        for key2, val2 in val1.items():
            if val2 == 0:
                edges_to_delete.append((key1, key2))
    for key1, key2 in edges_to_delete:
        g.remove_edge(key1, key2)

    # calc movement and disabled
    movement = {robot: [] for robot in robots}
    agents_names_to_agents = {str(agent) + '_o': agent for agent in agents}
    disabled = []
    for robot in robots:
        robot_name = str(robot)
        next = list(g.successors(robot_name))[0]
        while next != 't':
            if next[-1] == 'i':
                next = list(g.successors(next))[0]
            agent = agents_names_to_agents[next]
            disabled.append(agent)
            movement[robot].append(Point(agent.x, h))
            next = list(g.successors(next))[0]

    return {'movement': movement,
            'disabled': disabled}
