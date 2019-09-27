import dubins 
import numpy as np 
from abc import ABC, abstractmethod
import strategy.moviments as moviments
import sys
#sys.path.append("../..") # Adds higher directory to python modules path.
from statics import static_classes 
from statics.static_classes import world
from vision.pixel2metric import pixel2meters

ATT = 0
DEF = 1
SATT = 2

#TODO: encontrar valores otimos 
step = 0.001
centerGoal = np.array([0.72*static_classes.world.fieldSide,0])

class Entity(ABC):
    def __init__(self,name):
        self.host = None
        self.__target = np.array([0,0,0])
        self._path = None
        self.__name = name
        
    def __str__(self):
        return self.__name

    def possess(self, path, robot):
        self._path = path
        self.host = robot
        robot.target = self.__target
        robot.entity = self

    @abstractmethod
    def tatic(self,pose):
        pass

#update possession

class Attacker(Entity):
    def __init__(self):
        super().__init__("Atacante")
    def tatic(self, pose):
        self.__target = moviments.goToBallPlus(static_classes.world.ball.pos)
        return self.__target

class Goalkeeper(Entity):
    def __init__(self):
        super().__init__("Goleiro")
    def tatic(self, pose):
       self.__target =  moviments.goalkeep(static_classes.world.ball.pos, static_classes.world.ball.vel)
       return self.__target

class Defender(Entity):
    def __init__(self):
        super().__init__("Defensor")
    def tatic(self, pose):
        roboty = pose[1]
        self.__target = moviments.blockBallElipse(np.array(static_classes.world.ball.pos), np.array(static_classes.world.ball.vel), np.array(pose))
        return self.__target

class Midfielder(Entity):
    def __init__(self):
        super().__init__("MeioCampo")
    def tatic(self, pose):
        return np.array((0,0,0))

class TestPlayer(Entity):
    def __init__(self):
        super().__init__("TestPlayer")
    def tatic(self, pose):
        point = pixel2meters(world.mainPoint, (520,640))
        return (point[0],point[1],0)


class MovimentsDecider():
    def __init__(self):
        self.delta_ref = 0.1 * world.field_x_length
        self.ball_vmax = 1.5
        self.state = ATT
        self.listEntity = [TestPlayer(), Defender(), Attacker()]
        #self.listEntity = [TestPlayer(), TestPlayer(), TestPlayer()]
        self.turning_radius = 0.0375
        self.dynamicPossession = False

    def shortestTragectory(self, startPose, endPose, radius):
        altStartPose = (startPose[0], startPose[1], startPose[2] + np.pi)
        path = dubins.shortest_path(startPose, endPose, radius)
        path2 = dubins.shortest_path(altStartPose, endPose, radius)
        if(path.path_length() <= path2.path_length()):
            return path
        return path2
    
    @property
    def delta(self):
        d = self.delta_ref*(1 - abs(world.ball.inst_vx)/self.ball_vmax)
        if d < 0:
            print("Parece que o delta ficou negativo, a bola está rápida demais?")
            return 0
        return d
        
    def setFormation(self):
        if world.number_of_robots == 3:
            if world.ball.inst_x < -self.delta:
                if self.state != DEF:
                    self.listEntity = [Attacker(), Goalkeeper(), Defender()]
                self.state = DEF
            elif world.ball.inst_x > self.delta:
                if abs(world.gameScore) >= 8:
                    if self.state != SATT:
                        self.listEntity = [Attacker(), Midfielder(), Defender()]
                    self.state = SATT
                else:
                    if self.state != ATT:
                        self.listEntity = [Attacker(), Midfielder(), Goalkeeper()]
                    self.state = ATT

    def updadeHost(self):
        if self.dynamicPossession == False:
            for indx,robot in enumerate(static_classes.world.robots):
                if(indx >= len(self.listEntity)): break
                target = self.listEntity[indx].tatic(robot.pose)
                path = self.shortestTragectory(robot.pose, target, self.turning_radius) 
                self.listEntity[indx].possess(path, robot)
            return

        possessed = []
        for entity in self.listEntity:  
            minPath = None
            minCost = 1000
            host = None
            for indx,robot in enumerate(static_classes.world.robots):
                if robot in possessed:
                    continue
                target = entity.tatic(robot.pose)
                path = self.shortestTragectory(robot.pose, target, self.turning_radius) 
                cost = path.path_length()
                if cost < minCost:
                    minCost = cost
                    minPath = path
                    host = robot
            if host is not None:
                possessed.append(host)
                entity.possess(minPath, host)

    def calcPath(self):
        for robot in static_classes.world.robots:
            robot.entity.path = self.shortestTragectory(robot.pose, robot.entity.tatic(), self.turning_radius) 

#if __name__ == '__main__':
#    r = static_classes.world.robots[0]
#    ent = Attacker()
#    path = 0
#    ent.possess(path, 0)


