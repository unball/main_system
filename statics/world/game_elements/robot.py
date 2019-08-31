"""Robot class module."""
from statics.world.game_elements.element import Element


class Robot(Element):
    """Robot class child of Element()."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()
        self.__target =  np.array([0,0,0])
        self.__trajectory = [] 


    @property
    def target(self):
        return self.__target

    @property
    def trajectory(self):
        return self.__trajectory

    @target.setter
    def target(self,pose):
        self.__target = pose

    @trajectory.setter
    def trajectory(self, trajectory):
        self.__trajectory = trajectory



if __name__ == "__main__":
    robot = Robot()
