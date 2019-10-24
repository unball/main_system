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
        self.target2 = np.array([0,0,0])
        self.acceptableAngleError = 0.26

    def initialPose(self):
        return (0,0,0)
        
    def __str__(self):
        return self.__name

    def possess(self, path, robot):
        self._path = path
        self.host = robot
        robot.target = self.__target
        robot.entity = self

    @property
    def target(self):
        return self.__target

    @abstractmethod
    def tatic(self,pose):
        pass

#update possession

def spinKick(pose, host):
    if host is not None:
        distance = np.sqrt((pose[0]-static_classes.world.ball.pos[0])**2+(pose[1]-static_classes.world.ball.pos[1])**2)
        if distance < 0.07: 
            host.spin = -1 if pose[1]*world.fieldSide > 0 else 1
            host.spin = host.spin * np.sign(pose[0]-static_classes.world.ball.pos[0]) * world.fieldSide
        else: host.spin = 0

class Attacker(Entity):
    def __init__(self):
        super().__init__("Atacante")
        self.exploredAttack = False
        self.goalSide = 0
    def tatic(self, pose):
        self.chooseGoalSide()
        if static_classes.world.ball.x*world.fieldSide > 0.55: return moviments.followBally(static_classes.world.ball.pose, pose)
        self.__target = moviments.goToBallPlus(static_classes.world.ball.pos, pose, self.goalSide, self.host)
        self.target2 = self.__target
        return self.__target
    def initialPose(self):
        if world.getInitPos == -1:
            return (world.attacker_init_def_pos_x*world.fieldSide, 0, 0)
        if world.getInitPos == 1:
            return (0.04*world.fieldSide, 0, 0)
    def chooseGoalSide(self):
        if world.ball.x*world.fieldSide > 0:
            if self.exploredAttack == True:
                self.exploredAttack = False
                self.goalSide = 0.05*np.random.randint(-1,2)
        else:
            self.exploredAttack = True
                

class Goalkeeper(Entity):
    def __init__(self):
        super().__init__("Goleiro")
        self.acceptableAngleError = math.inf
    def tatic(self, pose):
        self.__target =  moviments.goalkeep(static_classes.world.ball.pos, static_classes.world.ball.vel, pose)
        self.target2 = self.__target
        spinKick(pose, self.host)
        return self.__target
    def initialPose(self):
        if world.getInitPos != 0:
            return (world.goalkeeper_init_pos_x*world.fieldSide, 0, np.pi/2)

class Defender(Entity):
    def __init__(self):
        super().__init__("Defensor")
    def tatic(self, pose):
        roboty = pose[1]
        self.__target = moviments.blockBallElipse(np.array(static_classes.world.ball.pos), np.array(static_classes.world.ball.vel), np.array(pose))
        self.target2 = self.__target
        #print("tatic: {0}".format(self.__target[2]))
        spinKick(pose, self.host)
        return self.__target
    def initialPose(self):
        if world.getInitPos != 0:
            return (world.defender_init_pos_x*world.fieldSide, 0, np.pi/2)

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
        self.listEntity = [Attacker(), Defender(), Goalkeeper()]
        self.turning_radius = statics.configFile.getValue("Turn_Radius", 0.070)
        self.dynamicPossession = False

    def endPoseDeltaTrajectory(self, startPose, endPose, radius, delta=1*np.pi/180):

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
            #outsidePoints = np.logical_and(abs(discretized) > [world.internal_limit_x, world.internal_limit_y], (abs(discretized) > np.array([mat.inf,world.internal_y_goal])) )
            outsidePoints = (abs(discretized[:,1]) > world.internal_y_goal)
            outsidePoints = np.logical_and(outsidePoints, np.logical_or(abs(discretized[:,0]) > world.internal_limit_x, abs(discretized[:,1]) > world.internal_limit_y))
            if np.sum(outsidePoints) == 0:
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

    def initialUpdateHost(self):
        for robot in static_classes.world.robots:
            if robot.entity is not None:
                path = self.shortestTragectory(robot.pose, robot.entity.initialPose(), self.turning_radius, robot) 
                robot.entity.possess(path, robot)


    def updadeHost(self):
        if self.dynamicPossession == False:
            for indx,robot in enumerate(static_classes.world.robots):
                if(indx >= len(self.listEntity)): break
                target = self.listEntity[indx].tatic(robot.pose)
                path = self.shortestTragectory(robot.pose, target, self.turning_radius, robot) 
                #if indx == 1: print("path: {0}".format(path.path_endpoint()))
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


