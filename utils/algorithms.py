import math
from typing import List, Tuple, Dict, Union

import networkx as nx
import numpy as np
from matplotlib import pyplot as plt
from networkx import path_weight
from scipy.optimize import linear_sum_assignment

from utils.functions import integrate_gauss, sigma_t, meeting_height
from world.agents.base_agent import BaseAgent
from world.agents.stochastic_agent import StochasticAgent
from world.robots.basic_robot import BasicRobot
from utils.point import Point
from utils.consts import Consts


def _can_stop_on_line(r: Tuple[float, float], a: Tuple[float, float], h: float, fv: float, v: float) -> bool:
    x_a, y_a = a
    x_r, y_r = r
    t_a = (h - y_a) / v
    t_r = math.sqrt((x_a - x_r) ** 2 + (h - y_r) ** 2) / fv
    return t_r <= t_a


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


def prev_iterative_assignment(robots: List['BasicRobot'], agents_copy: List['BaseAgent'], border) \
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

        optimal_assignment = linear_sum_assignment(distances)
        assigned_robots = optimal_assignment[0]
        assigned_agents = optimal_assignment[1]

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

    completion_time = max(free_time.values())
    return {'movement': movement,
            'completion_time': completion_time,
            'damage': expected_damage,
            'num_disabled': expected_num_disabled}


def iterative_assignment(robots: List['BasicRobot'], agents: List['BaseAgent'], border: float) \
        -> Dict[str, Union[Dict, int, float]]:
    agents = [StochasticAgent(loc=agent.loc, v=agent.v, sigma=0) for agent in agents]
    stats = stochastic_iterative_assignment(robots, agents, border)
    return {'movement': stats['movement'],
            'completion_time': stats['completion_time'],
            'damage': stats['expected_damage'],
            'num_disabled': stats['expected_num_disabled']}


def stochastic_iterative_assignment(robots: List['BasicRobot'], agents: List['StochasticAgent'], border: float) \
        -> Dict[str, Union[Dict, int, float]]:
    v = agents[0].v
    sigma = agents[0].sigma
    fv = robots[0].fv
    d = robots[0].d

    movement = {robot: [] for robot in robots}
    free_time = {robot: 0 for robot in robots}

    potential_damage = sum([border - a.y for a in agents])
    expected_num_disabled = 0
    expected_avoided_damage = 0

    agents_copy = [agent.clone() for agent in agents]

    # assign while there are agents alive
    while len(agents_copy) > 0:
        # calculate assignment costs
        utilities = [[] for _ in range(len(robots))]

        meeting_times = {robot: {agent: None for agent in agents_copy} for robot in robots}
        meeting_points = {robot: {agent: None for agent in agents_copy} for robot in robots}
        meeting_probs = {robot: {agent: None for agent in agents_copy} for robot in robots}

        for i, robot in enumerate(robots):
            # updated robot
            robot_at_time = robot.clone()
            if len(movement[robot]) > 0:
                robot_at_time.loc = movement[robots[i]][-1]

            for agent in agents_copy:
                # updated agent
                agent_at_time = agent.clone()
                agent_at_time.loc = Point(agent.x, agent.y + free_time[robot] * v)

                meeting_point = Point(agent.x, meeting_height(robot_at_time, agent_at_time))

                # if meeting outside the border, cost is inf
                if meeting_point.y > border:
                    utilities[i].append(0)
                    meeting_points[robot][agent] = None
                else:
                    dist = robot_at_time.loc.distance_to(meeting_point)
                    prob = round(integrate_gauss(mu=agent.x, sigma=sigma_t(sigma=sigma, t=meeting_point.y - agent.y),
                                                 left=agent.x - d, right=agent.x + d), 3)
                    utilities[i].append((border - meeting_point.y) * prob)

                    meeting_times[robot][agent] = dist / fv
                    meeting_points[robot][agent] = meeting_point
                    meeting_probs[robot][agent] = prob

        # apply optimal assignment
        optimal_assignment = linear_sum_assignment(utilities, maximize=True)
        assigned_robot_indexes = optimal_assignment[0]
        assigned_agent_indexes = optimal_assignment[1]

        # update according optimal assignment
        agents_to_remove = []
        non_assigned_num = 0
        for i_robot, i_agent in zip(assigned_robot_indexes, assigned_agent_indexes):
            assigned_robot = robots[i_robot]
            assigned_agent = agents_copy[i_agent]

            meeting_point = meeting_points[assigned_robot][assigned_agent]

            # if meeting outside the border, continue
            if meeting_point is None:
                non_assigned_num += 1
                continue

            # else, update values
            movement[assigned_robot].append(meeting_point)
            free_time[assigned_robot] += meeting_times[assigned_robot][assigned_agent]

            expected_avoided_damage += ((border - meeting_point.y) * meeting_probs[assigned_robot][assigned_agent])
            expected_num_disabled += meeting_probs[assigned_robot][assigned_agent]
            agents_to_remove.append(assigned_agent)

        if non_assigned_num == len(assigned_agent_indexes):
            break

        for a in agents_to_remove:
            agents_copy.remove(a)

    return {'movement': movement,
            'completion_time': max(free_time.values()),
            'expected_damage': potential_damage - expected_avoided_damage,
            'expected_num_disabled': expected_num_disabled}


def _delete_non_flow_edges(g: nx.DiGraph, flow) -> nx.DiGraph:
    edges_to_delete = []
    for key1, val1 in flow.items():
        for key2, val2 in val1.items():
            if val2 == 0:
                edges_to_delete.append((key1, key2))
    for key1, key2 in edges_to_delete:
        g.remove_edge(key1, key2)
    return g


def static_lack_moves(robots: List['BasicRobot'], agents: List['BaseAgent'], h: float):
    v = agents[0].v
    fv = robots[0].fv

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
            if _can_stop_on_line(r=robot.xy, a=agent.xy, h=h, fv=fv, v=v):
                g.add_edge(str(robot), str(agent) + '_i', weight=0, capacity=1)

    # add edges between agents
    for agent1 in agents:
        for agent2 in agents:
            if agent1 is agent2:
                continue
            if _can_stop_on_line(r=(agent1.x, h), a=(agent2.x, h - (agent1.y - agent2.y)), h=h, v=v, fv=fv):
                g.add_edge(str(agent1) + '_o', str(agent2) + '_i', weight=0, capacity=1)

    # add dummy source and target to use flow
    for robot in robots:
        g.add_edge('s', str(robot), weight=0, capacity=1)
        g.add_edge(str(robot), 't', weight=0, capacity=1)

    for agent in agents:
        g.add_edge(str(agent) + '_o', 't', weight=0, capacity=1)

    flow = nx.max_flow_min_cost(g, 's', 't')

    # delete all edges without flow
    g = _delete_non_flow_edges(g=g, flow=flow)

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


def single_static_lack_moves(robots: List['BasicRobot'], agents: List['BaseAgent'], h: float):
    v = agents[0].v
    fv = robots[0].fv
    robot = robots[0]

    g = nx.DiGraph()

    # create robot
    g.add_node(str(robot), pos=robot.xy, color='blue')

    # create agents divided to in and out
    for agent in agents:
        g.add_node(str(agent) + '_i', pos=(agent.x - 0.5, agent.y), color='red')
        g.add_node(str(agent) + '_o', pos=(agent.x + 0.5, agent.y), color='red')
        g.add_edge(str(agent) + '_i', str(agent) + '_o', weight=1)

    # add edges from robots to agents
    for agent in agents:
        if _can_stop_on_line(r=robot.xy, a=agent.xy, h=h, fv=fv, v=v):
            g.add_edge(str(robot), str(agent) + '_i', weight=0)

    # add edges between agents
    for agent1 in agents:
        for agent2 in agents:
            if agent1 is agent2:
                continue
            if _can_stop_on_line(r=(agent1.x, h), a=(agent2.x, h - (agent1.y - agent2.y)), h=h, v=v, fv=fv):
                g.add_edge(str(agent1) + '_o', str(agent2) + '_i', weight=0)

    for agent in agents:
        g.add_edge(str(agent) + '_o', 't', weight=0)

    path = nx.dag_longest_path(G=g)

    nodes_to_agents = {str(agent) + '_o': agent for agent in agents}

    return {'movement': {robot: [Point(nodes_to_agents[node].x, h) for node in path if node[-1] == 'o']},
            'disabled': [nodes_to_agents[node] for node in path if node[-1] == 'o']}


def stochastic_lack_moves(robots: List['BasicRobot'], agents: List['StochasticAgent'], h: float):
    v = agents[0].v
    fv = robots[0].fv
    d = robots[0].d
    sigma = agents[0].sigma
    agents = [a for a in agents if a.y < h]
    sigma_on_h = {agent: sigma_t(sigma=sigma, t=h - agent.y) for agent in agents}

    g = nx.DiGraph()

    # create robots
    for robot in robots:
        g.add_node(str(robot), pos=robot.xy)

    # create agents divided to in and out
    for agent in agents:
        # mu
        weight = round(integrate_gauss(mu=agent.x, sigma=sigma_on_h[agent], left=agent.x - d, right=agent.x + d), 3)

        g.add_node('mu_' + str(agent) + '_i')
        g.add_node('mu_' + str(agent) + '_o', pos=Point(agent.x, h), prob=weight, agent=agent)

        g.add_edge('mu_' + str(agent) + '_i', 'mu_' + str(agent) + '_o', weight=-weight * 1000, capacity=1)

        # +sigma
        weight = round(integrate_gauss(mu=agent.x, sigma=sigma_on_h[agent], left=agent.x - d + sigma_on_h[agent],
                                       right=agent.x + d + sigma_on_h[agent]), 3)

        g.add_node('ps_' + str(agent) + '_i')
        g.add_node('ps_' + str(agent) + '_o', pos=Point(agent.x + sigma_on_h[agent], h), prob=weight, agent=agent)

        g.add_edge('ps_' + str(agent) + '_i', 'ps_' + str(agent) + '_o', weight=-weight * 1000, capacity=1)

        # -sigma
        weight = round(integrate_gauss(mu=agent.x, sigma=sigma_on_h[agent], left=agent.x - d - sigma_on_h[agent],
                                       right=agent.x + d - sigma_on_h[agent]), 3)

        g.add_node('ms_' + str(agent) + '_i')
        g.add_node('ms_' + str(agent) + '_o', pos=Point(agent.x - sigma_on_h[agent], h), prob=weight, agent=agent)

        g.add_edge('ms_' + str(agent) + '_i', 'ms_' + str(agent) + '_o', weight=-weight * 1000, capacity=1)

    # add edges from robots to agents
    for robot in robots:
        for agent in agents:
            if _can_stop_on_line(r=robot.xy, a=agent.xy, h=h, v=v, fv=fv):
                g.add_edge(str(robot), 'mu_' + str(agent) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=robot.xy, a=(agent.x + sigma_on_h[agent], agent.y), h=h, v=v, fv=fv):
                g.add_edge(str(robot), 'ps_' + str(agent) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=robot.xy, a=(agent.x - sigma_on_h[agent], agent.y), h=h, v=v, fv=fv):
                g.add_edge(str(robot), 'ms_' + str(agent) + '_i', weight=0, capacity=1)

    # add edges between agents
    for agent1 in agents:
        for agent2 in agents:
            if agent1 is agent2:
                continue

            agent2_h = h - (agent1.y - agent2.y)
            agent2_sigma = sigma_t(sigma, agent2_h - agent2.y)

            if _can_stop_on_line(r=(agent1.x, h), a=(agent2.x, agent2_h), h=h, fv=fv, v=v):
                g.add_edge('mu_' + str(agent1) + '_o', 'mu_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x, h),
                                 a=(agent2.x + agent2_sigma, agent2_h), h=h, fv=fv, v=v):
                g.add_edge('mu_' + str(agent1) + '_o', 'ps_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x, h),
                                 a=(agent2.x - agent2_sigma, agent2_h), h=h, fv=fv, v=v):
                g.add_edge('mu_' + str(agent1) + '_o', 'ms_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x + sigma_on_h[agent1], h),
                                 a=(agent2.x, agent2_h), h=h, fv=fv, v=v):
                g.add_edge('ps_' + str(agent1) + '_o', 'mu_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x + sigma_on_h[agent1], h),
                                 a=(agent2.x + agent2_sigma, agent2_h), h=h, fv=fv, v=v):
                g.add_edge('ps_' + str(agent1) + '_o', 'ps_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x + sigma_on_h[agent1], h),
                                 a=(agent2.x - agent2_sigma, agent2_h), h=h, fv=fv, v=v):
                g.add_edge('ps_' + str(agent1) + '_o', 'ms_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x - sigma_on_h[agent1], h),
                                 a=(agent2.x, agent2_h), h=h, fv=fv, v=v):
                g.add_edge('ms_' + str(agent1) + '_o', 'mu_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x - sigma_on_h[agent1], h),
                                 a=(agent2.x + agent2_sigma, agent2_h), h=h, fv=fv, v=v):
                g.add_edge('ms_' + str(agent1) + '_o', 'ps_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x - sigma_on_h[agent1], h),
                                 a=(agent2.x - agent2_sigma, agent2_h), h=h, fv=fv, v=v):
                g.add_edge('ms_' + str(agent1) + '_o', 'ms_' + str(agent2) + '_i', weight=0, capacity=1)

    # add dummy source and target to use flow
    for robot in robots:
        g.add_edge('s', str(robot), weight=0, capacity=1)
        g.add_edge(str(robot), 't', weight=0, capacity=1)

    for agent in agents:
        g.add_edge('mu_' + str(agent) + '_o', 't', weight=0, capacity=1)
        g.add_edge('ps_' + str(agent) + '_o', 't', weight=0, capacity=1)
        g.add_edge('ms_' + str(agent) + '_o', 't', weight=0, capacity=1)

    flow = nx.max_flow_min_cost(g, 's', 't')
    _delete_non_flow_edges(g, flow)

    # calc movement and disabled
    movement = {robot: [] for robot in robots}
    pos_of_nodes = nx.get_node_attributes(g, 'pos')
    prob_of_nodes = nx.get_node_attributes(g, 'prob')
    agents_of_nodes = nx.get_node_attributes(g, 'agent')

    disabled = []
    exp_disabled = 0
    for robot in robots:
        robot_name = str(robot)
        next = list(g.successors(robot_name))[0]
        while next != 't':
            if next[-1] == 'i':
                next = list(g.successors(next))[0]
            disabled.append((agents_of_nodes[next], prob_of_nodes[next]))
            exp_disabled += prob_of_nodes[next]
            movement[robot].append(pos_of_nodes[next])
            next = list(g.successors(next))[0]

    return {'movement': movement,
            'disabled': disabled,
            'exp_disabled': exp_disabled}
