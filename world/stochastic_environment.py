from typing import List, Tuple, Union, cast

from world.environment import Environment
from world.robots.basic_robot import BasicRobot
from world.agents.stochastic_agent import StochasticAgent


class StochasticEnvironment(Environment):
    def __init__(self, robots: List[BasicRobot], agents: List[StochasticAgent], top_border: int, left_border: int,
                 right_border: int):
        super().__init__(robots, agents, top_border)
        self._top_border = top_border
        self._left_border = left_border
        self._right_border = right_border

    @property
    def top_border(self) -> int:
        return self._top_border

    @property
    def left_border(self) -> int:
        return self._left_border

    @property
    def right_border(self) -> int:
        return self._right_border

    def advance(self) -> bool:
        if self._step % 10 == 0:
            print(f'step {self._step}')

        self._step += 1

        for robot in self.robots:
            robot.advance()

        for agent in self.agents:
            agent.advance()
            if agent.x < self.left_border:
                agent.x = self.left_border
            elif agent.x > self.right_border:
                agent.x = self.right_border

            self._acc_damage += agent.v

        # check disablement and escaped
        for agent in self.agents:
            # if agent crosses border
            if agent.y >= self._border:
                self.agents.remove(agent)
                self._agents_escaped += 1
                print('agent escaped')
                continue

            for robot in self.robots:
                # if robot does not disable
                if not robot.is_disabling:
                    continue

                # the differences between the velocities can cause the robot
                # to jump over the agent without disabling it
                # thus the 1.5 factor which is greater than sqrt(2 range^2)
                if agent.loc.distance_to(robot.loc) <= 1.4 * robot.d:
                    self.agents.remove(agent)
                    self._agents_disabled += 1
                    print('agent disabled')
                    break

        return len(self.agents) == 0

