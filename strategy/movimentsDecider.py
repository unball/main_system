import dubins 
import numpy as np 
from abc import ABC, abstractmethod
import strategy.moviments as moviments
import sys
import statics.configFile
#sys.path.append("../..") # Adds higher directory to python modules path.
from statics import static_classes 
from statics.static_classes import world
from vision.pixel2metric import pixel2meters
import math

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
        self.acceptableAngleError = 0.26
        
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
        self.__target = moviments.goToBallPlus(static_classes.world.ball.pos, pose)
        return self.__target

class Goalkeeper(Entity):
    def __init__(self):
        super().__init__("Goleiro")
        self.acceptableAngleError = math.inf
    def tatic(self, pose):
       self.__target =  moviments.goalkeep(static_classes.world.ball.pos, static_classes.world.ball.vel, pose)
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
        point = pixel2meters(world.mainPoint, (350,471))
        return (point[0],point[1],0)


class MovimentsDecider():
    def __init__(self):
        self.delta_ref = 0.1 * world.field_x_length
        self.ball_vmax = 1.5
        self.state = ATT
        #self.listEntity = [Attacker(), Attacker(), TestPlayer()]
        self.listEntity = [Goalkeeper(), Goalkeeper(), TestPlayer()]
        self.turning_radius = statics.configFile.getValue("Turn_Radius", 0.070)
        self.dynamicPossession = False

    def endPoseDeltaTrajectory(self, startPose, endPose, radius, delta=45*np.pi/180):

        startPose = (startPose[0], startPose[1], startPose[2])
        endPoseMax = (endPose[0], endPose[1], endPose[2] + delta)
        endPoseMin = (endPose[0], endPose[1], endPose[2] - delta)

        pathCenter = dubins.shortest_path(startPose, endPose, radius)
        pathMax = dubins.shortest_path(startPose, endPoseMax, radius)
        pathMin = dubins.shortest_path(startPose, endPoseMin, radius)

        return [pathCenter, pathMax, pathMin]

    def trajectoryCost(self, trajectory, robot):
        if trajectory is None:
            return math.inf
        
        angleCost = trajectory.path_endpoint()[2]-robot.th

        return trajectory.path_length()#**2+angleCost**2

    def chooseMinTrajectory(self, trajectoryList, radius, robot):
        #if trajectoryList[0].path_length() > np.pi*radius:
        #    return trajectoryList[0]
        if len(trajectoryList) == 0: return None
        path = min(trajectoryList, key=lambda x: self.trajectoryCost(x, robot))

        return path

    def filterTrajectories(self, trajectoryList):
        filtered = []
        for trajectory in trajectoryList:
            discretized = np.array(trajectory.sample_many(0.01)[0])[:,:2]
            insidePoints = abs(discretized) > [world.field_x_length/2*0.93, world.field_y_length/2]
            if np.sum(insidePoints[:,0] | insidePoints[:,1]) == 0:
                filtered.append(trajectory)
        
        return filtered
        

    def shortestTragectory(self, startPose, endPose, radius, robot):
        altStartPose = (startPose[0], startPose[1], startPose[2] + np.pi)

        trajectories = self.endPoseDeltaTrajectory(startPose, endPose, radius)
        altTrajectories = self.endPoseDeltaTrajectory(altStartPose, endPose, radius)
        
        trajectoriesInsideField = self.filterTrajectories(trajectories)
        altTrajectoriesInsideField = self.filterTrajectories(altTrajectories)

        best = self.chooseMinTrajectory(trajectoriesInsideField, radius, robot)
        altBest = self.chooseMinTrajectory(altTrajectoriesInsideField, radius, robot)

        bestCost = self.trajectoryCost(best, robot)
        altBestCost = self.trajectoryCost(altBest, robot)
        
        HIST = 0.1
        diff = bestCost-altBestCost
        #print(diff)

        if(robot.dir == 1):
            if(diff > HIST):
                robot.dir = -1
                return altBest
            return best
        else:
            if(diff < -HIST):
                robot.dir = 1
                return best
            return altBest


        return best

        altTrajectories = self.endPoseDeltaTrajectory(altStartPose, endPose, radius)

        HIST = 0.2
        diff = trajectories[0].path_length()-altTrajectories[0].path_length()

        if(robot.dir == 1):
            if(diff > HIST):
                robot.dir = -1
                return self.chooseMinTrajectory(altTrajectories, radius)
            return self.chooseMinTrajectory(trajectories, radius)
        else:
            if(diff < -HIST):
                robot.dir = 1
                return self.chooseMinTrajectory(trajectories, radius)
            return self.chooseMinTrajectory(altTrajectories, radius)



        return path

        path2 = dubins.shortest_path(altStartPose, endPose, radius)
        # # !TODO: NÃO ALTERAR robot.dir aqui
        HIST = 0.09
        diff = path.path_length()-path2.path_length()

        if abs(robot.w) > np.pi/2 and np.linalg.norm(np.array(robot.vel)) > 0.1:
            print("oi fui executado")
            if(robot.dir == 1): return path
            else: return path2
        
        if(robot.dir == 1):
            if(diff > HIST):
                print("troquei para 2")
                robot.dir = -1
                return path2
            return path
        else:
            if(diff < -HIST):
                print("troquei para 1")
                robot.dir = 1
                return path
            return path2

        # if(path.path_length() <= path2.path_length()):
        #     print("frente")
        #     robot.dir = 1
        #     return path
        # print("trás")
        # robot.dir = -1
        # return path2
    
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
                path = self.shortestTragectory(robot.pose, target, self.turning_radius, robot) 
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
                path = self.shortestTragectory(robot.pose, target, self.turning_radius, robot.dir) 
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


