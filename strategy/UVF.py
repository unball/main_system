import numpy as np
import random
import cv2
import math
import time

from strategy.src2.un_field import univectorField




RADIUS = .08
KR = .49
K0 = 0.12
DMIN = .05
LDELTA = 0.045


def ObstaclesArea(campo):
    if (campo == -1):
        obstaclesList = 0.01*np.array([[65, 0],[70, 0], [65, -10], [70, -10], [65, -20], [70, -20], [65, 10], [70, 10], [65, 20], [70, 20], [65, 30], [70, 30], [65, -30], [70, -30]])
        vObstacle = np.array([[0 , 0], [0 , 0], [0 , 0], [0 , 0], [0 , 0], [0,0], [0,0], [0,0], [0,0], [0 , 0], [0 , 0], [0 , 0], [0 , 0], [0 , 0], [0,0], [0,0]])
    elif (campo == 1):
        obstaclesList = (-1) * 0.01* np.array([[65, 0],[70, 0], [65, -10], [70, -10], [65, -20], [70, -20], [65, 10], [70, 10], [65, 20], [70, 20], [65, 30], [70, 30], [65, -30], [70, -30]])
        vObstacle = np.array([[0 , 0], [0 , 0], [0 , 0], [0 , 0], [0 , 0], [0,0], [0,0], [0,0], [0,0], [0 , 0], [0 , 0], [0 , 0], [0 , 0], [0 , 0], [0,0], [0,0]])
    else:
        obstaclesList = np.array([])
        vObstacle = np.array([])


    return obstaclesList, vObstacle


def getPath(start, end, univetField):
    currentPos = start

    newPos = None
    step = .02
    beta = .01

    t0 = time.time()

    path = []
    #while(np.linalg.norm(currentPos - end) >= beta):
    #       pass

    theta = univetField.getVec(_robotPos=currentPos, _vRobot=[0,0])
    v = np.array([math.cos(theta), math.sin(theta)])
    newPos = currentPos + (step*v)

    if (time.time() - t0 > .05):
        return False, newPos
    
    angle = theta if theta < np.pi else theta-np.pi 
    path.append((newPos[0],newPos[1],angle))

    currentPos = newPos


    return path[0]
    
def calcPath(start, end, goal, SIDE):
    # obstaclesList = []
    # obstacle    = np.array([])
    # vObstacle   = np.array([])
    
    obstacle, vObstacle = ObstaclesArea(SIDE)

    univetField = univectorField(attack_goal=goal, _rotation=True)
    univetField.updateConstants(RADIUS, KR, K0, DMIN, LDELTA)

    univetField.updateBall(end)

    univetField.updateObstacles(obstacle, vObstacle)

    path = getPath(start, end, univetField)

    return path

def UVFPath(SIDE, robot, ball):
    t = time.time()

    robot = np.array([.3,.3])
    ball = np.array([.1,-.2])
    gol = np.array([.75*SIDE,0])
    p = calcPath(robot, ball, gol, SIDE)

    virtualBall = np.array([.65*SIDE, 0])
    yProj = virtualBall[1]-(ball[1]-virtualBall[1])/(virtualBall[0]-ball[0]) * (.75*SIDE-virtualBall[0])
    yProj = max(min(yProj, .15), -.15)

    p2 = calcPath(ball, virtualBall,np.array([.75*SIDE, yProj]), SIDE)

    return (p, p2)

if __name__ == "__main__":
    pass

    # print(p)
    # print('-------')
    # print(p2)


    # print("--------")
    # print(time.time()-t)