import math
from typing import List, Tuple

import networkx as nx
import numpy as np
from matplotlib import pyplot as plt
from networkx import path_weight

from world.agents.base_agent import BaseAgent
from world.agents.stochastic_agent import StochasticAgent
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


def stochastic_lack_moves(robots: List['BasicRobot'], agents: List['StochasticAgent'], h: float):
    v = agents[0].v
    fv = robots[0].fv
    sigma = agents[0].sigma

    g = nx.DiGraph()

    # create robots
    for robot in robots:
        g.add_node(str(robot), pos=robot.xy, color='blue')

    # create agents divided to in and out
    for agent in agents:
        key_points = []
        for key_point in key_points
        g.add_node(str(agent) + '_i', pos=(agent.x - 0.5, agent.y), color='red')
        g.add_node(str(agent) + '_o', pos=(agent.x + 0.5, agent.y), color='red')
        g.add_edge(str(agent) + '_i', str(agent) + '_o', weight=-1, capacity=1)

    # add edges from robots to agents
    for robot in robots:
        for agent in agents:
            # if can_stop_on_line(r=robot.xy, a=agent.xy, h=h):
            g.add_edge(str(robot), str(agent) + '_i', weight=0, capacity=1)

    # add edges between agents
    for agent1 in agents:
        for agent2 in agents:
            if agent1 is agent2:
                continue
            # if can_stop_on_line(r=(agent1.x, h), a=(agent2.x, h - (agent1.y - agent2.y)), h=h):
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