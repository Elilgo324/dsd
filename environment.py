from time import sleep
from typing import List

from agents.base_agent import BaseAgent
from robots.base_robot import BasicRobot
from utils.consts import Consts
from copy import deepcopy


class Environment:
    def __init__(self, robots: List[BasicRobot], agents: List[BaseAgent]):
        self._agents = agents
        self._robots = robots
        self._acc_damage = 0
        self._steps = 0

    @property
    def robots(self) -> List[BasicRobot]:
        return self._robots

    @property
    def agents(self) -> List[BaseAgent]:
        return self._agents

    def clone_robots(self) -> List[BasicRobot]:
        return deepcopy(self._robots)

    def clone_agents(self) -> List[BaseAgent]:
        return deepcopy(self._agents)

    def get_robot_i(self, i: int) -> BasicRobot:
        return self.robots[i]

    def advance(self) -> bool:
        self._steps += 1

        if Consts.DEBUG:
            sleep(1)
            print(f'*** step: {self._steps} ***')
            print(f'#robots: {len(self.robots)}')
            print(f'#agents: {len(self.agents)}')
            print(f'acc damage: {self._acc_damage}')

        for robot in self.robots:
            robot.advance()

        for agent in self.agents:
            agent.advance()
            self._acc_damage += 1

        # check disablement
        for agent in self.agents:
            for robot in self.robots:
                if agent.loc.distance_to(robot.loc) < Consts.DISABLEMENT_RANGE:
                    self.agents.remove(agent)
                    print('agent disabled')
                    break

        return len(self.agents) == 0

    def results(self) -> str:
        return f'*** results *** ' \
               f'\n steps: {self._steps} \t\t accumulated damage: {round(self._acc_damage, 2)}'