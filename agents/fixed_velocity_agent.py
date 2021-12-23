from agents.base_agent import BaseAgent
from utils.point import Point
from utils.consts import Consts


class FixedVelocityAgent(BaseAgent):
    def advance(self):
        self.loc = Point(self.x, self.y + self.v)
