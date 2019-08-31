"""Robot class module."""
import numpy as np
from statics.world.game_elements.element import Element


class Robot(Element):
    """Robot class child of Element()."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()
        self.__trajectory = [] 
        self.entity = None

    def discretize(self, step):
        self.trajectory =  self.entity.__path.sample_many(step)



    @property
    def trajectory(self):
        return self.__trajectory


    @trajectory.setter
    def trajectory(self, trajectory):
        self.__trajectory = trajectory

    


if __name__ == "__main__":
    robot = Robot()
