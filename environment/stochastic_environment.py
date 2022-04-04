from copy import deepcopy
from typing import List, Tuple, Union

import numpy as np

from environment.environment import Environment
from environment.robots.basic_robot import BasicRobot
from environment.agents.stochastic_agent import StochasticAgent


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

    def generate_PA(self):
        left, up, right = self.agents[0].advance_distribution

        # init 3d matrix of grid rows, cols and time
        PA = np.zeros((self.top_border, self.top_border, self.right_border))

        # in t=0 we have prob. 1 of being in the initial loc
        for agent in self.agents:
            PA[0][int(agent.y)][int(agent.x)] = 1

        # filling the rest times by dp
        T = int(self.top_border)
        num_rows = int(self.top_border)
        num_cols = int(self.right_border)

        for t in range(1, T):
            # fill the rows by dp
            for row in range(1, num_rows):
                # handle extreme cells
                PA[t][row][0] = (left + up) * PA[t - 1][row - 1][0] + left * PA[t - 1][row - 1][1]
                PA[t][row][int(num_cols) - 1] = (right + up) * PA[t - 1][row - 1][int(num_cols) - 1] + right * \
                                                PA[t - 1][row - 1][int(num_cols) - 2]

                for col in range(1, int(num_cols) - 1):
                        PA[t][row][col] += left * PA[t - 1][row - 1][col + 1] \
                                           + up * PA[t - 1][row - 1][col] \
                                           + right * PA[t - 1][row - 1][col - 1]

        return PA

    def generate_UA(self, PA=None):
        if PA is None:
            PA = self.generate_PA()

        T, num_rows, num_cols = PA.shape
        UA = np.zeros(PA.shape)

        for t in range(T):
            for r in range(num_rows):
                for c in range(num_cols):
                    # the utility is the prevented damage for this cell multiplied by the expected agents amount
                    UA[t, r, c] = (self.top_border - r) * PA[t, r, c]

        return UA

    def generate_U(self, UA=None):
        if UA is None:
            UA = self.generate_UA()

        T, num_rows, num_cols = UA.shape
        U = np.zeros((T, int(num_rows / 2), int(num_cols / 2)))
        T, num_rows, num_cols = U.shape

        for t in range(T):
            for r in range(num_rows):
                for c in range(num_cols):
                    U[t][r][c] = UA[t][2 * r][2 * c] + UA[t][2 * r + 1][2 * c] + UA[t][2 * r][2 * c + 1] + \
                                 UA[t][2 * r + 1][2 * c + 1]

        return U