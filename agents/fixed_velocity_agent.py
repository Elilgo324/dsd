from agents.base_agent import BaseAgent
from utils.point import Point
from utils.consts import Consts


class FixedVelocityAgent(BaseAgent):
    def advance(self):
        prev_loc = self.loc
        self.loc = Point(self.x, self.y + self.v)

        if Consts.DEBUG:
            print(f'{str(self)}: {prev_loc} -> {self.loc}')
