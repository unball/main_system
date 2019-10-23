"""Robot class module."""
import numpy as np
from statics.world.game_elements.element import Element
import numpy as np


class Robot(Element):
    """Robot class child of Element()."""

    def __init__(self):
        """Responsible to instantiate the attributes of the parent class."""
        super().__init__()
        self.__trajectory = [] 
        self.entity = None
        self.spin = 0
        self.dir = 1
        self.nearTarget = False

    def pathLength(self):
        if self.nearTarget == True:
            return self.distanceToTarget()
        if self.entity is not None and self.entity._path is not None:
            return self.entity._path.path_length()
        return 0

    def nextStep(self):
        if len(self.__trajectory) != 0 and len(self.__trajectory[0]) >= 1:
            pose = self.__trajectory[0][1]
            angle = pose[2] - 2*np.pi if pose[2] > np.pi else pose[2]
            return (pose[0], pose[1], angle)
        # !TODO: Decidir o que fazer quando não há uma trajetória
        else: return (0,0,0)

    def distanceToTarget(self):
        r = self.__trajectory[0][0]
        rtar = self.__trajectory[0][-1]
        return np.sqrt((r[0]-rtar[0])**2 + (r[1]-rtar[1])**2)

    def distanceToTargetAngle(self):
        r = self.__trajectory[0][0]
        rtar = self.__trajectory[0][-1]
        return abs(r[2]-rtar[2])

    def discretize(self, step):
        if self.entity is not None and self.entity._path is not None:
            self.__trajectory =  self.entity._path.sample_many(step)
            
            #if self.entity.__str__() == "Defensor": print("d: {0}".format(self.distanceToTarget()))

            # Chegou no target!!!!
            if self.entity.__str__() == "Atacante":
                self.nearTarget = False
                pass
            else:
                if self.distanceToTarget() < 0.06:# and self.distanceToTargetAngle() < self.entity.acceptableAngleError:
                    self.nearTarget = True
                    atualPose = self.__trajectory[0][0]
                    angle = self.__trajectory[0][-1][2]
                    newPose = (atualPose[0], atualPose[1], angle)
                    self.__trajectory = [[newPose, newPose]]
                else: self.nearTarget = False

    @property
    def trajectory(self):
        return self.__trajectory


    @trajectory.setter
    def trajectory(self, trajectory):
        self.__trajectory = trajectory

    


if __name__ == "__main__":
    robot = Robot()
