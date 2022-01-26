from copy import deepcopy
from typing import List, Tuple, Union

from environment.agents.base_agent import BaseAgent
from environment.robots.basic_robot import BasicRobot


class Environment:
    def __init__(self, robots: List[BasicRobot], agents: List[BaseAgent], border: int):
        self._agents = agents
        self._robots = robots
        self._border = border
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
    def border(self) -> int:
        return self._border

    @property
    def step(self) -> int:
        return self._step

    @property
    def acc_damage(self) -> float:
        return self._acc_damage

    @property
    def agents_disabled(self) -> float:
        return self._agents_disabled

    @property
    def agents_escaped(self) -> float:
        return self._agents_escaped

    def clone_robots(self) -> List[BasicRobot]:
        return deepcopy(self._robots)

    def clone_agents(self) -> List[BaseAgent]:
        return deepcopy(self._agents)

    def get_robot_i(self, i: int) -> BasicRobot:
        return self.robots[i]

    def advance(self) -> bool:
        if self._step % 10 == 0:
            print(f'step {self._step}')

        self._step += 1

        for robot in self.robots:
            robot.advance()

        for agent in self.agents:
            agent.advance()
            self._acc_damage += agent.v

        # check disablement and escaped
        for agent in self.agents:
            if agent.y > self._border:
                self.agents.remove(agent)
                self._agents_escaped += 1
                print('agent escaped')
                break

            for robot in self.robots:
                # if robot does not disable
                if not robot.is_disabling:
                    continue

                # the differences between the velocities can cause the robot
                # to jump over the agent without disabling it
                # thus the 1.5 factor which is greater than sqrt(2 range^2)
                if agent.loc.distance_to(robot.loc) <= 1.4 * robot.r:
                    self.agents.remove(agent)
                    self._agents_disabled += 1
                    print('agent disabled')
                    break

        return len(self.agents) == 0

    def stats(self) -> str:
        return f'steps: {self._step}' \
               f'   acc damage: {round(self._acc_damage, 2)}' \
               f'   agents disabled: {self._agents_disabled}' \
               f'   agents escaped: {self._agents_escaped}'
