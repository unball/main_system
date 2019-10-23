import numpy as np
import math
from random import randint
from statics.static_classes import world

#TODO: implementar spin

def projectBall(ballPos, ballVel, dt=0.03):
    dt = 3*world.timeInterval
    return (ballPos[0]+dt*ballVel[0], ballPos[1]+dt*ballVel[1])

def goToBallPlus(ballPos, robotPose, goalSide):
    ballPos = projectBall(ballPos, world.ball.vel)
    finalTarget = np.array([.75*world.fieldSide, goalSide])
    ballTarget = finalTarget-ballPos  
    ballRobot = ballPos-robotPose[:2]


    if abs(ballPos[1]) > 0.9*world.field_y_length:
        if world.fieldSide == -1: angle = 0
        else: angle = np.pi
    else: angle = np.arctan2(ballTarget[1],ballTarget[0])

    distance = np.linalg.norm(ballRobot)
    if distance < 0.08 and robotPose[0]*world.fieldSide > (ballPos[0])*world.fieldSide+0.03*world.fieldSide and abs(robotPose[1]-ballPos[1]) < 0.03:
        return (finalTarget[0], finalTarget[1], angle)
    
    return (ballPos[0], ballPos[1], angle)

def followBally(rb, rr):
    angle = np.pi/2
    if rr[1] > rb[1]: angle = -np.pi/2
    
    return (0.40*world.fieldSide, rb[1], angle)

def insideEllipse(r, a, b, rm):
    return ((r[0]-rm[0])/a)**2+((r[1]-rm[1])/b)**2 < 1

def blockBallElipse(rb, vb, rr):
    #return followBally(rb, rr)
    rb = projectBall(rb, vb)
    a = 0.3
    b = 0.4
    rm = np.array([0.75*world.fieldSide, 0])
    e = np.array([1/a, 1/b])
    finalTarget = np.array([.75*world.fieldSide, 0])

    if insideEllipse(rb, 0.25, 0.33, rm):
        vb = finalTarget-(rb[0], -rb[1])
    else: vb = finalTarget-rb
    
    coefficients = [sum((vb*e)**2), 2*np.dot((rb-rm)*e, vb*e), sum(((rb-rm)*e)**2)-1]
    
    try:
        roots = np.roots(coefficients)
        u = min(roots)
    except:
        return followBally(rb, rr)
        
    #print(roots)
    
    if u.imag == 0:
        r = rb + u*vb
        r_ = r-rm
        o = math.atan2(r_[1], r_[0])
        t = math.atan2(a*math.sin(o), b*math.cos(o))
        r_ort = (-a*math.sin(t), b*math.cos(t))
        r_ort_angle = math.atan2(r_ort[1], r_ort[0])
        
        if rr[1] > r[1] and r_ort_angle > 0: r_ort_angle = r_ort_angle+np.pi
        if rr[1] < r[1] and r_ort_angle < 0: r_ort_angle = r_ort_angle+np.pi
        
        return (r[0], r[1], r_ort_angle)
    
    return followBally(rb, rr)

#def blockBallElipse(goal, ballPos, ballVel, roboty):
#    try:
#        ########definir z = [a**2, b**2]###########
#        # TODO: Corrigir problema da velocidade 0
#        z = np.array([.4**2, .2**2])
#        ballPos = np.array(ballPos)
#        ballVel = np.array(ballVel)
#        c =  [sum((ballVel**2)*z), 2*np.dot(z*ballVel, ballPos-goal), sum(z*(ballPos-goal)**2) - np.prod(z)]
#        k = min(np.roots(c))
#        if k.imag == 0:
#            target = k*ballVel + ballPos
#            goalTarget = target-goal
#            angle = np.arctan2(-ballVel[1],ballVel[0])
#            if target[1]-roboty > 0:
#                angle = angle*(-1)
#            return (target[1],target[0], angle)
#        return (.15*world.fieldSide, ballPos[1]+.1, np.pi/2 )
#    except:
#        return (0,0,0)

def goalkeep(ballPos, ballVel, robotPose):
    xGoal = world.fieldSide * .68
    #testar velocidade minima (=.15?)
    if ((ballVel[0]*world.fieldSide) > .1) and  ((ballPos[0]*world.fieldSide)> .15):
        #verificar se a projeção está no gol
        #projetando vetor até um xGoal-> y = (xGoal-Xball) * Vyball/Vxball + yBall 
        ytarget = max(min((((xGoal-ballPos[0])/ballVel[0])*ballVel[1])+ballPos[1],0.23),-0.23)
        angle = np.pi/2 if robotPose[1] < ytarget else -np.pi/2
        return (xGoal, ytarget, angle)
    #Se não acompanha o y
    ytarget = max(min(ballPos[1],0.23),-0.23)
    angle = np.pi/2 if robotPose[1] < ytarget else -np.pi/2
    return np.array([xGoal, ytarget, angle])

def mirrorPos(posDom):
    radiusMin = .1                      #raio minimo para simetria
    mirrorCenter = np.array([.375*world.fieldSide, .0]) #centro da simetria radial
    rectangle = world.fieldSide * np.array([[.1,.4], [.55, -.4]])
    centerDom =  mirrorCenter -  posDom[0]
    #se o robô dominante estiver denforatro do raio minimo, usa a simetria
    if np.linalg.norm(centerDom) > radiusMin:
        targetMid = centerDom + mirrorCenter
        return (targetMid, posDom[1])
    #leva o robô passivo pra aresta Y do retangulo mais proxima mantento o X do robô
    if np.array(self.pos[argMid][0])[0][1] >= 0:
        targetMid[0] = np.array(self.pos[argMid][0])[0][0]
        targetMid[1] = rectangle[0][1]
    else:
        targetMid[0] = np.array(self.pos[argMid][0])[0][0]
        targetMid[1] = rectangle[1][1]
    return (targetMid,0)
