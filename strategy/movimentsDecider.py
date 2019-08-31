import dubins 
import numpy as np 
from abc import ABC, abstractmethod
from statics import static_classes 
import moviments



#TODO: encontrar valores otimos 
turning_radius = 0.0375
step = 0.001


class Entity(ABC):
    def __init__(self):
        self.host = None
        self.__target = np.array([0,0,0])
        self.__path = None
    def possess(path, id):
        self.__path = path
        self.host = id
        static_classes.world.robots[id].trajectory = self.__path.sample_many(step)

    @abstractmethod
    def tatic(self,pose):
        pass
#update possession

class Attacker(Entity):
    def __init__():
        super().__init__()
    def tatic(self, pose):
        pass

class Goalkeeper(Entity):
    def __init__():
        super().__init__()
    def tatic(self, pose):
       self.__target =  moviments.goalkeep()
       return self.__target

class Defender(Entity):
    def __init__():
        super().__init__()
    def tatic(self, pose):
        pass

class Midfielder(Entity):
    def __init__(self):
        super().__init__()
    def tatic(self, pose):
        pass


class MovimentsDecider():
    def __init__(self):
        self.listEntity = []

    def shortestTragectory(startPose, endPose, radius):
        altStartPose = (startPose[0], startPose[1], startPose[2] + np.pi)
        path = dubins.shortest_path(startPose, endPose, radius)
        path2 = dubins.shortest_path(altStartPose, endPose, radius)
        if(path.path_length() <= path2.path_length()):
            return path
        return path2

    def updadePossesion():
        possessed = []
        for entity self.listEntity:
            
            minPath = None
            minPathLength = 1000
            host = None
            for indx,robot in enumerate(static_classes.world.robots):
                if indx in possessed:
                    continue
                target = entity.tatic(robot.pose)
                path = shortestTragectory(robot.pose, target, turning_radius) 
                length = path.path_length()
                if length < minPathLength:
                    minPathLength = length
                    minPath = path
                    host = indx
                    possessed.append(indx)
            entity.possession(minPath, host)
