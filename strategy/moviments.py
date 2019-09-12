import numpy as np
from random import randint

#TODO: implementar spin

fieldSide = -1
finalTarget = np.array([.75*fieldSide, randint(-1, 1) *.2])

def goToBallPlus(ballPos):
    ballTarget = finalTarget - ballPos  
    return (ballPos[0], ballPos[1], np.arctan2(ballTarget[1],ballTarget[0]))

def blockBallElipse(goal, ballPos, ballVel):
    try:
        ########definir z = [a**2, b**2]###########
        # TODO: Corrigir problema da velocidade 0
        z = np.array([.4**2, .2**2])
        ballPos = np.array(ballPos)
        ballVel = np.array(ballVel)
        c =  [sum((ballVel**2)*z), 2*np.dot(z*ballVel, ballPos-goal), sum(z*(ballPos-goal)**2) - np.prod(z)]
        k = min(np.roots(c))
        if k.imag == 0:
            target = k*ballVel + ballPos
            goalTarget = target-goal
            return (target[1],target[0], np.pi/2 + np.arctan2(goalTarget[1],goalTarget[0]))
        return (.15*fieldSide, (ballPos[1]+.1, np.pi/2) )
    except:
        return (0,0,0)
def goalkeep(ballPos, ballVel):
    xGoal = fieldSide * .72
    #testar velocidade minima (=.15?)
    if ((ballVel[0]*fieldSide) > .15) and  ((ballPos[0]*fieldSide)> .15):
        #verificar se a projeção está no gol
        #projetando vetor até um xGoal-> y = (xGoal-Xball) * Vyball/Vxball + yBall 
        return (xGoal, (((xGoal-ballPos[0])/ballVel[0])*ballVel[1])+ballPos[1], np.pi/2)
    #Se não acompanha o y
    return np.array([xGoal, ballPos[1], np.pi/2])

    def mirrorPos(posDom):
        radiusMin = .1                      #raio minimo para simetria
        mirrorCenter = np.array([.375*fieldSide, .0]) #centro da simetria radial
        rectangle = fieldSide * np.array([[.1,.4], [.55, -.4]])
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
