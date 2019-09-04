import dubins 
import numpy as np 
from abc import ABC, abstractmethod
import strategy.moviments as moviments
import sys
#sys.path.append("../..") # Adds higher directory to python modules path.
from statics import static_classes 



#TODO: encontrar valores otimos 
turning_radius = 0.0375
step = 0.001
centerGoal = np.array([0.72*static_classes.world.fieldSide,0])

class Entity(ABC):
    def __init__(self):
        self.host = None
        self.__target = np.array([0,0,0])
        self.__path = None

    def possess(self, path, id):
        self.__path = path
        self.host = id
        static_classes.world.robots[id].target = self.__target
        static_classes.world.robots[id].entity = self

    @abstractmethod
    def tatic(self,pose):
        pass

#update possession

class Attacker(Entity):
    def __init__(self):
        super().__init__()
    def tatic(self, pose):
        self.__target = moviments.goToBallPlus(static_classes.world.ball.pos)
        return self.__target

class Goalkeeper(Entity):
    def __init__(self):
        super().__init__()
    def tatic(self, pose):
       self.__target =  moviments.goalkeep(static_classes.world.ball.pos, static_classes.world.ball.vel)
       return self.__target

class Defender(Entity):
    def __init__(self):
        super().__init__()
    def tatic(self, pose):
        self.__target = moviments.blockBallElipse(centerGoal, static_classes.world.ball.pos,static_classes.world.ball.vel)
        return self.__target

class Midfielder(Entity):
    def __init__(self):
        super().__init__()
    def tatic(self, pose):
        pass


class MovimentsDecider():
    def __init__(self):
        self.listEntity = [Goalkeeper(), Defender(), Attacker()]

    def shortestTragectory(self, startPose, endPose, radius):
        altStartPose = (startPose[0], startPose[1], startPose[2] + np.pi)
        path = dubins.shortest_path(startPose, endPose, radius)
        path2 = dubins.shortest_path(altStartPose, endPose, radius)
        if(path.path_length() <= path2.path_length()):
            return path
        return path2

    def updadeHost(self):
        possessed = []
        for entity in self.listEntity:  
            minPath = None
            minCost = 1000
            host = -1
            for indx,robot in enumerate(static_classes.world.robots):
                if indx in possessed:
                    continue
                target = entity.tatic(robot.pose)
                path = self.shortestTragectory(robot.pose, target, turning_radius) 
                cost = path.path_length()
                if cost < minCost:
                    minCost = cost
                    minPath = path
                    host = indx
                    possessed.append(indx)
            entity.possess(minPath, host)

    def calcPath(self):
        for robot in static_classes.world.robots:
            robot.entity.path = self.shortestTragectory(robot.pose, robot.entity.tatic(), turning_radius) 

if __name__ == '__main__':
    r = static_classes.world.robots[0]
    ent = Attacker()
    path = 0
    ent.possess(path, 0)


