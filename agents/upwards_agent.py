from agents.base_agent import BaseAgent
from utils.point import Point

class UpwardsAgent(BaseAgent):
    def advance(self):
        prev_loc = self.loc
        self.loc = Point(self.x,self.y+self.v)
        print(f'{str(self)}: {prev_loc} -> {self.loc}')