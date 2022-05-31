import math
from typing import List, Tuple

import networkx as nx
import numpy as np
from matplotlib import pyplot as plt
from networkx import path_weight

from world.agents.base_agent import BaseAgent
from world.robots.basic_robot import BasicRobot
from utils.point import Point
from utils.consts import Consts


def static_lack_moves(robots: List['BasicRobot'], agents: List['BaseAgent'], h: float):
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


def stochastic_lack_moves(robots: List['BasicRobot'], row: float, U, PA):
    R = U[:, row, :]
    T, n_cols = R.shape
    # f = robots[0].fv

    g = nx.DiGraph()

    # create robots
    for robot in robots:
        g.add_node(str(robot), color='blue', pos=np.array([robot.x, robot.y]))

    # create utility cells
    for t in range(T):
        for c in range(n_cols):
            g.add_node(f'[{t},{c}]_i', time=t, col=c, color='red', pos=np.array([2 * c, 10 + 2 * (5 * t)]))
            g.add_node(f'[{t},{c}]_o', time=t, col=c, color='red', pos=np.array([2 * c, 10 + 2 * (5 * t + 3)]))

            # the weight needs to be an integer
            if R[t, c] != 0:
                g.add_edge(f'[{t},{c}]_i', f'[{t},{c}]_o', weight=-int(10_000 * R[t, c]), capacity=1)

    # add edges between cells
    for t in range(T - 1):
        # handle first cell in row, edge in the same column gets a tiny bonus
        g.add_edge(f'[{t},{0}]_o', f'[{t + 1},{0}]_i', weight=-1, capacity=len(robots))
        g.add_edge(f'[{t},{0}]_i', f'[{t + 1},{0}]_i', weight=-1, capacity=len(robots))

        g.add_edge(f'[{t},{0}]_o', f'[{t + 1},{1}]_i', weight=0, capacity=len(robots))
        g.add_edge(f'[{t},{0}]_i', f'[{t + 1},{1}]_i', weight=0, capacity=len(robots))

        g.add_edge(f'[{t},{0}]_o', f'[{t + 1},{2}]_i', weight=0, capacity=len(robots))
        g.add_edge(f'[{t},{0}]_i', f'[{t + 1},{2}]_i', weight=0, capacity=len(robots))

        # handle last cell in row, edge in the same column gets a tiny bonus
        g.add_edge(f'[{t},{n_cols - 1}]_o', f'[{t + 1},{n_cols - 1}]_i', weight=-1, capacity=len(robots))
        g.add_edge(f'[{t},{n_cols - 1}]_i', f'[{t + 1},{n_cols - 1}]_i', weight=-1, capacity=len(robots))

        g.add_edge(f'[{t},{n_cols - 1}]_o', f'[{t + 1},{n_cols - 2}]_i', weight=0, capacity=len(robots))
        g.add_edge(f'[{t},{n_cols - 1}]_i', f'[{t + 1},{n_cols - 2}]_i', weight=0, capacity=len(robots))

        g.add_edge(f'[{t},{n_cols - 1}]_o', f'[{t + 1},{n_cols - 3}]_i', weight=0, capacity=len(robots))
        g.add_edge(f'[{t},{n_cols - 1}]_i', f'[{t + 1},{n_cols - 3}]_i', weight=0, capacity=len(robots))

        # handle the rest, edge in the same column gets a tiny bonus
        for c in range(1, n_cols - 1):
            g.add_edge(f'[{t},{c}]_o', f'[{t + 1},{c}]_i', weight=-1, capacity=len(robots))
            g.add_edge(f'[{t},{c}]_i', f'[{t + 1},{c}]_i', weight=-1, capacity=len(robots))

            g.add_edge(f'[{t},{c}]_o', f'[{t + 1},{c - 1}]_i', weight=0, capacity=len(robots))
            g.add_edge(f'[{t},{c}]_o', f'[{t + 1},{c + 1}]_i', weight=0, capacity=len(robots))

            g.add_edge(f'[{t},{c}]_i', f'[{t + 1},{c - 1}]_i', weight=0, capacity=len(robots))
            g.add_edge(f'[{t},{c}]_i', f'[{t + 1},{c + 1}]_i', weight=0, capacity=len(robots))

    # for each column and robot, add edge corresponding to their arrival time
    for robot in robots:
        for c in range(n_cols):
            arrival_time = int(Point(c, row).distance_to(robot.loc) / robot.fv)
            g.add_edge(str(robot), f'[{arrival_time},{c}]_i', weight=0, capacity=1)

    # add dummy source and target to use flow
    g.add_node('s', color='orange', pos=np.array([n_cols, 0]))
    g.add_node('t', color='orange', pos=np.array([n_cols, 12 * T]))

    for robot in robots:
        g.add_edge('s', str(robot), weight=0, capacity=1)

    for c in range(n_cols):
        g.add_edge(f'[{T - 1},{c}]_o', 't', weight=0, capacity=len(robots))
        g.add_edge(f'[{T - 1},{c}]_i', 't', weight=0, capacity=len(robots))

    flow = nx.max_flow_min_cost(g, 's', 't')

    # delete all edges without flow
    edges_to_delete = []
    for key1, val1 in flow.items():
        for key2, val2 in val1.items():
            if val2 == 0:
                edges_to_delete.append((key1, key2))
    for key1, key2 in edges_to_delete:
        g.remove_edge(key1, key2)

    nodes_times = nx.get_node_attributes(g, 'time')
    nodes_cols = nx.get_node_attributes(g, 'col')

    # calc movement and disabled
    active_time = 0
    movement = {robot: [] for robot in robots}
    timing = {robot: [] for robot in robots}
    expected_disabled = 0
    expected_utility = 0
    for robot in robots:
        next = str(robot)

        while True:
            next = list(g.successors(next))[0]
            if next == 't':
                break

            if next[-1] == 'o':
                cur_time = nodes_times[next]
                cur_col = nodes_cols[next]
                expected_disabled += PA[cur_time][row][cur_col]
                expected_utility += U[cur_time][row][cur_col]
                active_time = max(active_time, cur_time)

            elif next[-1] == 'i':
                movement[robot].append(Point(nodes_cols[next], row))
                timing[robot].append(nodes_times[next])

    # edge_labels = {k: v / 10_000 for k, v in nx.get_edge_attributes(g, 'weight').items() if v != 0}
    # pos = nx.get_node_attributes(g, 'pos')
    # plt.figure(3, figsize=(10, 20))
    # nx.draw(g, pos=pos,
    #         node_color=nx.get_node_attributes(g, 'color').values(),
    #         with_labels=True, node_size = [0.5 for _ in g.nodes])
    # nx.draw_networkx_edge_labels(g, pos, edge_labels=edge_labels, rotate=False)
    # plt.show()

    return {'movement': movement,
            'timing': timing,
            'utility': expected_utility,
            'active_time': active_time,
            'expected_disabled': expected_disabled}