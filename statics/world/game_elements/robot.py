"""Robot class module."""
import numpy as np
from statics.world.game_elements.element import Element
import statics.static_classes
import numpy as np
import math


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
        self.step = 0.035
        self.radius = 0.06

    def pathLength(self):
        if self.nearTarget == True:
            return self.distanceToTarget()
        if self.entity is not None and self.entity._path is not None:
            return self.entity._path.path_length()
        return 0

    def nextStep(self):
        if len(self.__trajectory) != 0 and len(self.__trajectory[0]) > 1:
            pose = self.__trajectory[0][1]
            angle = pose[2] - 2*np.pi if pose[2] > np.pi else pose[2]
            return (pose[0], pose[1], angle)
        # !TODO: Decidir o que fazer quando não há uma trajetória
        else: 
            return self.pose

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
            if self.entity.__str__() == "Atacante":
                try:
                    t = (statics.static_classes.world.field_x_length/2-self.x)/(statics.static_classes.world.ball.x-self.x)
                except:
                    t = math.inf
                yProj = self.y+(statics.static_classes.world.ball.y-self.y)*t
                if self.entity._path.segment_length(0) < np.pi*15/180*self.radius and ((self.entity._path.segment_length(1) > np.pi*180/180*self.radius or self.th) or abs(yProj) < 0.15):
                    #normalized = 0.1*self.entity._path.segment_length(1)/(self.entity._path.path_length()-self.entity._path.segment_length(1))
                    normalized = self.entity._path.segment_length(1)/self.entity._path.path_length()
                    #if self.entity.__str__() == "Atacante": print("adsf: " + str(normalized))
                    step = min(6*step*max(normalized,0.5), self.entity._path.path_length()/2.001)

            self.__trajectory =  self.entity._path.sample_many(step)
            
            #if self.entity.__str__() == "Defensor": print("d: {0}".format(self.distanceToTarget()))
            
            # Chegou no target!!!!
            if self.entity.__str__() == "Atacante":
                self.nearTarget = False
            else:
                if self.distanceToTarget() < 0.06:# and self.distanceToTargetAngle() < self.entity.acceptableAngleError:
                    self.nearTarget = True
                    atualPose = self.pose
                    angle = self.entity.target2[2]
                    #if self.entity.__str__() == "Defensor": print(angle)
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
