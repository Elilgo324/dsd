from typing import List, Tuple

from agents.base_agent import BaseAgent
from robots.basic_robot import BasicRobot
from utils.consts import Consts
from copy import deepcopy


class Environment:
    def __init__(self, robots: List[BasicRobot], agents: List[BaseAgent], world_size: Tuple[int, int]):
        self._agents = agents
        self._robots = robots
        self._world_size = world_size
        self._acc_damage = 0
        self._step = 0
        self._agents_disabled = 0
        self._agents_escaped = 0

    @property
    def robots(self) -> List[BasicRobot]:
        return self._robots

    @property
    def agents(self) -> List[BaseAgent]:
        return self._agents

    @property
    def world_size(self) -> Tuple[int, int]:
        return self._world_size

    @property
    def step(self) -> int:
        return self._step

    def clone_robots(self) -> List[BasicRobot]:
        return deepcopy(self._robots)

    def clone_agents(self) -> List[BaseAgent]:
        return deepcopy(self._agents)

    def get_robot_i(self, i: int) -> BasicRobot:
        return self.robots[i]

    def advance(self) -> bool:
        self._step += 1

        if Consts.DEBUG:
            print(f'*** step: {self._step} ***')
            print(f'#robots: {len(self.robots)}')
            print(f'#agents: {len(self.agents)}')
            print(f'acc damage: {self._acc_damage}')

        for robot in self.robots:
            robot.advance()

        for agent in self.agents:
            agent.advance()
            self._acc_damage += agent.v

        # check disablement and escaped
        for agent in self.agents:
            for robot in self.robots:
                if agent.loc.distance_to(robot.loc) <= robot.r + 0.1:
                    self.agents.remove(agent)
                    self._agents_disabled += 1
                    if Consts.DEBUG:
                        print('agent disabled')
                    break
                if agent.y > self._world_size[1]:
                    self.agents.remove(agent)
                    self._agents_escaped += 1
                    if Consts.DEBUG:
                        print('agent escaped')
                    break

        return len(self.agents) == 0

    def stats(self) -> str:
        return f'steps: {self._step}' \
               f'   acc damage: {round(self._acc_damage, 2)}' \
               f'   agents disabled: {self._agents_disabled}' \
               f'   agents escaped: {self._agents_escaped}'
