from world.agents.base_agent import BaseAgent
from utils.point import Point


class DeterministicAgent(BaseAgent):
    def advance(self) -> None:
        self.loc = Point(self.x, self.y + self.v)
