import math
from typing import List, Tuple

import networkx as nx
import numpy as np
from matplotlib import pyplot as plt
from networkx import path_weight

from utils.functions import integrate_gauss, future_sigma
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


def _delete_non_flow_edges(g: nx.DiGraph, flow: nx.DiGraph) -> nx.DiGraph:
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
            if _can_stop_on_line(r=robot.xy, a=agent.xy, h=h):
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


def stochastic_lack_moves(robots: List['BasicRobot'], agents: List['StochasticAgent'], h: float):
    v = agents[0].v
    fv = robots[0].fv
    d = robots[0].d
    sigma = agents[0].sigma
    agents = [a for a in agents if a.y < h]
    fsigma = {agent: future_sigma(sigma=sigma, stride=int(h - agent.y)) for agent in agents}

    g = nx.DiGraph()

    # create robots
    for robot in robots:
        g.add_node(str(robot), pos=robot.xy)

    # create agents divided to in and out
    for agent in agents:
        # mu
        weight = integrate_gauss(mu=agent.x, sigma=fsigma[agent], left=agent.x - d, right=agent.x + d)

        g.add_node('mu_' + str(agent) + '_i')
        g.add_node('mu_' + str(agent) + '_o', pos=Point(agent.x, h), prob=weight, agent=agent)

        g.add_edge('mu_' + str(agent) + '_i', 'mu_' + str(agent) + '_o', weight=-weight * 1000, capacity=1)

        # +sigma
        weight = integrate_gauss(
            mu=agent.x, sigma=fsigma[agent], left=agent.x - d + fsigma[agent], right=agent.x + d + fsigma[agent])

        g.add_node('ps_' + str(agent) + '_i')
        g.add_node('ps_' + str(agent) + '_o', pos=Point(agent.x + fsigma[agent], h), prob=weight, agent=agent)

        g.add_edge('ps_' + str(agent) + '_i', 'ps_' + str(agent) + '_o', weight=-weight * 1000, capacity=1)

        # -sigma
        weight = integrate_gauss(mu=agent.x, sigma=future_sigma(sigma=fsigma[agent], stride=int(
            h - agent.y)), left=agent.x - d - fsigma[agent], right=agent.x + d - fsigma[agent])

        g.add_node('ms_' + str(agent) + '_i')
        g.add_node('ms_' + str(agent) + '_o', pos=Point(agent.x - fsigma[agent], h), prob=weight, agent=agent)

        g.add_edge('ms_' + str(agent) + '_i', 'ms_' + str(agent) + '_o', weight=-weight * 1000, capacity=1)

    # add edges from robots to agents
    for robot in robots:
        for agent in agents:
            if _can_stop_on_line(r=robot.xy, a=agent.xy, h=h, v=v, fv=fv):
                g.add_edge(str(robot), 'mu_' + str(agent) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=robot.xy, a=(agent.x + fsigma[agent], agent.y), h=h, v=v, fv=fv):
                g.add_edge(str(robot), 'ps_' + str(agent) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=robot.xy, a=(agent.x - fsigma[agent], agent.y), h=h, v=v, fv=fv):
                g.add_edge(str(robot), 'ms_' + str(agent) + '_i', weight=0, capacity=1)

    # add edges between agents
    for agent1 in agents:
        for agent2 in agents:
            if agent1 is agent2:
                continue

            agent_2_relative_loc = (agent2.x, h - (agent1.y - agent2.y))
            if _can_stop_on_line(r=(agent1.x, h), a=agent_2_relative_loc, h=h, fv=fv, v=v):
                g.add_edge('mu_' + str(agent1) + '_o', 'mu_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x, h), a=agent_2_relative_loc, h=h, fv=fv, v=v):
                g.add_edge('mu_' + str(agent1) + '_o', 'ps_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x, h), a=agent_2_relative_loc, h=h, fv=fv, v=v):
                g.add_edge('mu_' + str(agent1) + '_o', 'ms_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x + fsigma[agent1], h), a=agent_2_relative_loc, h=h, fv=fv, v=v):
                g.add_edge('ps_' + str(agent1) + '_o', 'mu_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x + fsigma[agent1], h), a=agent_2_relative_loc, h=h, fv=fv, v=v):
                g.add_edge('ps_' + str(agent1) + '_o', 'ps_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x + fsigma[agent1], h), a=agent_2_relative_loc, h=h, fv=fv, v=v):
                g.add_edge('ps_' + str(agent1) + '_o', 'ms_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x - fsigma[agent1], h), a=agent_2_relative_loc, h=h, fv=fv, v=v):
                g.add_edge('ms_' + str(agent1) + '_o', 'mu_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x - fsigma[agent1], h), a=agent_2_relative_loc, h=h, fv=fv, v=v):
                g.add_edge('ms_' + str(agent1) + '_o', 'ps_' + str(agent2) + '_i', weight=0, capacity=1)

            if _can_stop_on_line(r=(agent1.x - fsigma[agent1], h), a=agent_2_relative_loc, h=h, fv=fv, v=v):
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
            disabled.append(agents_of_nodes[next])
            exp_disabled += prob_of_nodes[next]
            movement[robot].append(pos_of_nodes[next])
            next = list(g.successors(next))[0]

    return {'movement': movement,
            'disabled': disabled,
            'exp_disabled': exp_disabled}
