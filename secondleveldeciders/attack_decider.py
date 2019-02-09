"""Attack decider module."""

from secondleveldeciders.secondleveldecider import SecondLvlDecider

from players.striker import Striker
from players.goalkeeper import Goalkeeper
from players.defender import Defender

from membership_functions import *
import numpy as np
from megafunctions import fuzzy
from megafunctions import defuzzy
from math import sin
from random import randint
from math import acos

magic_number = 14
LEFT = -1
RIGHT = 1

class AttackDecider(SecondLvlDecider):
    """Class docstring."""

    def __init__(self):
        self.pos = np.matrix([[.0,0],[0,0],[0,0]])
        self.vel = np.matrix([[.0,0],[0,0],[0,0]])
        self.ballPos = np.array([.0, .0])
        self.ballVel = np.array([.0, .0])
        self.targets = np.matrix([[.0,0],[0,0],[0,0]])
        self.per_robot = []
        self.formationS = "GDS"
        self.game_score = .0
        self.finalTarget = np.array([.75, 0])
        self.bounds = np.array([-.25, .25])
        self.fieldSide = 1

    #1
    def setup(self, world):
        self.defineBounds(world.gameScore)
        self.definePosVel(world)
        self.defineTarget(world.fieldSide)
    #2
    def setFormation(self, world):
        self.id_formation(world.gameScore)
        self.rearrange_formation(world)
        print(self.formationS)
        print(self.per_robot)

    #refazer fuzzy, usar megafunctions.py
    '''Rearrange the list of players.
        """FAM é a matrix de especialista """
        FAM = np.matrix([[-1, -.9, -.2, 0, .4],[-.6,-.2,0,.6,.8],[.2,.4,.8,.9,1]])
        """tops_dist sao os locais de tops da megafunction dist """
        tops_dist = np.array([.35,.50,.85,1.20,1.35])
        """tops_dist sao os locais de tops da megafunction ori """
        tops_ori = np.array([0, .5, 1])'''
    def id_formation(self, gameScore):
        """Identify the formation based on the world state."""
        score = self.FUZZYscore(gameScore)#linear procurar melhor fuzzy
        print("score: ", score)
        self.game_score = gameScore/10.0
        if type(score) == type([]):
            self.formation = score
            return None
        ball_x = magic_number * self.FUZZYball_x(self.ballPos[0])
        total_score = ball_x + score
        self.formation, self.formationS = self.defuzzicator(total_score)

    def FUZZYscore(self, gameScore):
        """Docstring."""
        #super attack
        if gameScore <= -7 or gameScore >= 7:
            self.formationS = 'DSS'
            return [Defender(), Striker(), Striker()]
        elif gameScore > -7 and gameScore <= -5:
            FA = i_neg(-7, -5, gameScore)
            D = i_pos(-7, -5, gameScore)
            return (FA * (-7)) + (D * (-5))
        elif gameScore > -5 and gameScore <= 0:
            D = i_neg(-5, 0, gameScore)
            N = i_pos(-5, 0, gameScore)
            return (D * (-5)) + (N * (0))
        elif gameScore > 0 and gameScore <= 5:
            N = i_neg(0, 5, gameScore)
            A = i_pos(0, 5, gameScore)
            return (N * (0)) + (A * (5))
        elif gameScore > 5 and gameScore <= 7:
            A = i_neg(5, 7, gameScore)
            FA = i_pos(5, 7, gameScore)
            return (A * (5)) + (FA * (7))

    def FUZZYball_x(self, ballX):
        """Docstring."""
        if ballX < -0.5:
            return -0.5
        elif ballX > 0.5:
            return 0.5
        elif ballX > -0.5 and ballX <= -0.25:
            SD = i_neg(-0.5, -0.25, ballX)
            D = i_pos(-0.5, -0.25, ballX)
            return (SD * (-0.5)) + (D * (-0.25))
        elif ballX > -0.25 and ballX <= 0:
            D = i_neg(-0.25, 0, ballX)
            N = i_pos(-0.25, 0, ballX)
            return (D * (-0.25)) + (N * (0))
        elif ballX > 0 and ballX <= 0.25:
            N = i_neg(0, 0.25, ballX)
            A = i_pos(0, 0.25, ballX)
            return (N * (0)) + (A * (0.25))
        elif ballX > 0.25 and ballX <= 0.5:
            A = i_neg(0.25, 0.5, ballX)
            SA = i_pos(0.25, 0.5, ballX)
            return (A * (0.25)) + (SA * (0.5))

    def defuzzicator(self, score):
        """Docstring."""
        print(score)
        if score <= -7:
            return ([Goalkeeper(), Defender(), Defender()], "GDD")
        elif score > -7 and score <= 3.5:
            return ([Goalkeeper(), Defender(), Striker()], "GDS")
        elif score > 3.5 and score <= 10.5:
            return ([Goalkeeper(), Striker(), Striker()], "GSS")
        else:
            # score > 10.5
            return ([Defender(), Striker(), Striker()], "DSS")

    def maxi(self,per, jf):
        maxval  = -2.0
        maxi = 0
        for i in range(0,3):
            if per[i]>maxval and jf[i]!=-1:
                maxval =per[i]
                maxi = i
        return maxi

    def rearrange_formation(self, world):
        """Rearrange the list of players."""
        """FAM é a matrix de especialista """
        FAM = np.matrix([[-1, -.9, -.2, 0, .4],[-.6,-.2,0,.6,.8],[.2,.4,.8,.9,1]])
        """tops_dist sao os locais de tops da megafunction dist """
        tops_dist = np.array([.35,.50,.85,1.20,1.35])
        """tops_dist sao os locais de tops da megafunction ori """
        tops_ori = np.array([0, .5, 1])

        """ pos,vel e ball são as posições dos robos, velocidade dos robos e posição da bola respectivamente"""
        ball = self.ballPos
        """Distancia bola-robo"""
        dist_BR = ball - self.pos
        abs_dist_BR = np.sqrt(np.power(dist_BR,2).sum(1))
        """Distancia bola-robo normalizada"""
        dist_BR = dist_BR/abs_dist_BR

        abs_vel = np.sqrt(np.power(self.vel,2).sum(1))
        """velocidade robos normalizada"""
        self.vel = self.vel/abs_vel
        """orientacao pra fuzzy ori"""
        ori = np.multiply(self.vel,dist_BR).sum(1)

        """Distancia pro centro do nosso gol-robo (harcoded) - MUDAR"""
        dist_CG = np.array([.75,.0]) - self.pos
        dist_CG = np.sqrt(np.power(dist_CG,2).sum(1))
        print(dist_CG[0,0])
        """per_robot pertinencia ao ataque de cada robo (-1,+1)"""
        self.per_robot = []
        for i in range(0,3):
            fuzzy_dist = fuzzy(dist_CG[i,0],tops_dist[:-1],tops_dist, tops_dist[1:])
            fuzzy_ori = fuzzy(ori[i,0],tops_ori[:-1],tops_ori, tops_ori[1:])
            self.per_robot.append(defuzzy(fuzzy_ori,FAM,fuzzy_dist))
        """rearranjando formação..."""
        new_formation = [Defender(),Defender(),Defender()]
        seek = [Striker(), Defender(), Goalkeeper()]
        ja_foi = [0, 0, 0]
        jfnf = [0,0,0]              #ja foi nao foi
        for k in range(0, 3):
            for i in range(0, 3):
                if (seek[k].id == self.formation[i].id and ja_foi[i] != -1):
                    maxi = self.maxi(self.per_robot,  jfnf)
                    new_formation[maxi] = seek[k]
                    ja_foi[i] = -1
                    jfnf[maxi] = -1
        return new_formation

    # se for ataque retorna true 
    def pair_attack(self):
        ball_vel_x = self.ballVel[0]
        ball_x = self.ballPos[0]
        if self.fieldSide == LEFT:
            if ball_x > self.bounds[1]:
                    return True #ataque
            elif  ball_x > self.bounds[0] and ball_x <= self.bounds[1]:
                if self.formationS == "GDS" or self.formationS == "GSS":
                    if ball_vel_x >= 0:
                        return True #ataque
                elif self.formationS=="DSS":
                    return True #ataque
        else:
            self.bounds = (self.game_score * .25) +  np.array([-.25,.25])
            if ball_x < self.bounds[1]:
                    return True #ataque
            elif  ball_x > self.bounds[0] and ball_x <= self.bounds[1]:
                if self.formationS == "GDS" or self.formationS == "GSS":
                    if ball_vel_x >= 0:
                        return True #ataque
                elif self.formationS=="DSS":
                    return True #ataque
        return False
        
    #separa os indices dos robôs por pertinencia
    def robotArgs(self, perRobot):
        argMax = perRobot.argmax()
        argMin = perRobot.argmin()
        argmid = 0
        for i in range(3):
            if (i != argMax) and (i != argMin):
                argmid = i
                break
        return argMax, argmid, argMin

    #garante que o target do robô está dentro do retangulo da simetria
    def insideRectangle(self, point, rectangle):
        if point[0] < rectangle[0][0]:
            point[0] = rectangle[0][0]

        if point[1] < rectangle[1][1]:
            point[1] = rectangle[1][1]

        if point[0] > rectangle[1][0]:
            point[0] = rectangle[1][0]

        if point[1] > rectangle[0][1]:
            point[1] = rectangle[0][1]

        return point
    
    #simetria radial de ataque
    def mirrorPos(self, argMax, argMid):
        mirrorCenter = np.array([.375, .0]) #centro da simetria radial
        radiusMin = .1                      #raio minimo para simetria
        rectangle = np.array([[.1,.4], [.55, -.4]])
        if self.fieldSide == RIGHT:   #se o lado de defesa for o direito troca os valores
            mirrorCenter = np.array([-.375, .0])
            rectangle = np.array([[-.55, .4], [-.1, -.4]])
        centerDom = mirrorCenter - np.array(self.pos[argMax])[0] #centro do espelho - posição do dominante
        #se o robô dominante estiver denforatro do raio minimo, usa a simetria
        if np.linalg.norm(centerDom) > radiusMin:
            targetMid = centerDom + mirrorCenter
            return self.insideRectangle(targetMid, rectangle)
        #leva o robô passivo pra aresta Y do retangulo mais proxima mantento o X do robô
        if np.array(self.pos[argMid][0])[0][1] >= 0:
            targetMid[0] = np.array(self.pos[argMid][0])[0][0]
            targetMid[1] = rectangle[0][1]
        else:
            targetMid[0] = np.array(self.pos[argMid][0])[0][0]
            targetMid[1] = rectangle[1][1]
        return targetMid

    #calcula posição da projeção da bola do semicirculo de defesa
    def blockBallRadius(self, radius=.375):
        #radius = .375
        goalCenter = np.array([-.75, .0])
        if self.fieldSide == RIGHT:
            goalCenter[0] = .75
        #Baskara
        if (self.ballVel[0]*self.fieldSide) > .15:
            a = 1 + ((self.ballVel[1]/self.ballVel[0])**2)

            b = (-2*goalCenter[0])
            b = b - (2*self.ballPos[0]*((self.ballVel[1]/self.ballVel[0])**2)) 
            b = b + (2*((self.ballVel[1]/self.ballVel[0])**2)*self.ballPos[1])

            c = (goalCenter[0]**2) + ((self.ballPos[0]**2)*((self.ballVel[1]/self.ballVel[0])**2))
            c = c - (2*self.ballPos[0]*(self.ballVel[1]/self.ballVel[0])*self.ballPos[1])
            c = c + (self.ballPos[1]**2) - (radius**2)

            x = np.array([.0, .0])
            x[0] = (-b + (((b**2)-(4*a*c))**.5))/(2*a)
            x[1] = (-b - (((b**2)-(4*a*c))**.5))/(2*a)
            if x[0].imag == 0:
                #x conhecido
                xConh = x[x.argmax()]
                if self.world.fieldSide == RIGHT:
                    xConh = x[x.argmin()]
                
                #constante
                k = (xConh - self.ballPos[0])/self.ballVel[0]

                #y conhecido
                yConh = (k*self.ballVel[1]) + self.ballPos[1]
                return np.array([xConh, yConh])
        
        if not self.ballInsideArea():
            k = radius/(np.linalg.norm(self.ballPos - goalCenter))
            return k*(self.ballPos - goalCenter) + goalCenter
        
        return np.array([(.15*self.fieldSide), (self.ballPos[1]+.1)])
    
    def ballInsideArea(self):
        if abs(self.ballPos[0]) > .15:
            return False
        elif abs(self.ballPos[1]) > .35:
            return False
        return True

    #calcula posição da projeção da bola do semicirculo de defesa
    def blockBallElipse(self,Goal):
        ########definir z = [a**2, b**2]###########
        if (self.ballVel[0]*self.fieldSide) > .15:
            z = np.array([.4**2, .2**2])
            c =  [sum((self.ballVel**2)*z), 2*np.dot(z*self.ballVel, self.ballPos-Goal), sum(z*(self.ballPos-Goal)**2) - np.prod(z)]
            k = min(np.roots(c))
            if k.imag == 0:
                target = k*self.ballVel + self.ballPos
                if(not self.ballInsideArea()):
                    return target
                target[1] = target[1] + .1
                return target
        #apenas fica entre a bola e o Goal
        if not self.ballInsideArea():
            return (np.sqrt(np.prod(z)/sum(z*(self.ballPos-Goal))) * (self.ballPos-Goal)) + Goal
    
        return np.array([(.15*self.fieldSide), (self.ballPos[1]+.1)])

    #calcula target do defender quando se está atacando
    def midFielder(self, shooter):
        #colocar o X no mid bound inferior?
        boundLine = min(self.bounds)
        if self.fieldSide == RIGHT:
            boundLine = max(self.bounds)
        target = np.array([boundLine, -self.targets[shooter, 1]])
        if abs(self.targets[shooter, 1]) < .20:
            target[1] = .20
            if self.targets[shooter, 1] > 0:
                target[1] = -.20 
        return target

    #atualiza os 3 targets
    def updateTargets(self):
        perRobot = np.array(self.per_robot)
        argMax, argMid, argMin = self.robotArgs(perRobot)
        #shooter
        self.targets[argMax] = self.shoot(argMax) #filtrar possibilidade de estar na area

        #assistente e defensor
        attack = self.pair_attack()
        if attack and ((self.formationS == "GSS") or (self.formationS == "DSS")):
            self.targets[argMid] = self.mirrorPos(argMax, argMid)
        elif attack and ((self.formationS == "GDS") or (self.formationS == "GDD")):
            self.targets[argMid] = self.midFielder(argMax)
        else:
            self.targets[argMid] = self.blockBallRadius()

        #goleiro e ultimo defensor
        if self.formationS == "GSS" or \
           self.formationS == "GDS" or \
           self.formationS == "GDD" or \
           self.ballPos[0]*self.fieldSide > .20:
            self.targets[argMin] = self.goalkeep()
        else:
            self.targets[argMin] = self.blockBallRadius()
        
        return self.targets
        
    #calcula o target robô levar a bola ao gol
    def shoot(self, shooter):
        robotBall = np.array(self.ballPos - self.pos[shooter])[0]  
        ballTarget = self.finalTarget - np.array(self.ballPos)[0]
        dot = np.dot(robotBall, ballTarget)
        normrobotBall = np.linalg.norm(robotBall)
        normballTarget = np.linalg.norm(ballTarget) 
        try:
            fi = acos(dot/(normrobotBall*normballTarget))
        except:
            return self.finalTarget
        distEball = .25*sin(fi)
        if distEball < .05:
            return self.finalTarget
        elif normrobotBall < .05 :
            return self.finalTarget

        r1 = normballTarget + distEball
        xe = distEball*self.finalTarget[0]+ r1*self.ballPos[0]/(r1+distEball)
        ye = distEball*self.finalTarget[1]+ r1*self.ballPos[1]/(r1+distEball)
        return np.array([xe,ye])
            
    #define o target final (no gol adiversario)
    def defineTarget(self, fieldSide):
        self.fieldSide = fieldSide
        if self.ballVel[0] < (.005*fieldSide): 
            self.finalTarget =  np.array([.75*fieldSide, randint(-1, 1) *.2])

    def defineBounds(self, gameScore):
        self.bounds = - (gameScore * .025) +  np.array([-.25,.25])

    def definePosVel(self, world):
        robots = world.robots
        ball = world.ball
        for i in range(0,3):
            self.pos[i] = np.array(robots[i].pos)
            self.vel[i] = np.array(robots[i].vel)
        self.ballPos = ball.pos
        self.ballVel = ball.vel

    def goalkeep(self):
        xGoal = self.fieldSide * .72
        #testar velocidade minima (=.15?)
        if ((self.ballVel[0]*self.fieldSide) > .15) and \
           ((self.ballPos*self.fieldSide)> .15):
           #verificar se a projeção está no gol
           #projetando vetor até um xGoal-> y = (xGoal-Xball) * Vyball/Vxball + yBall 
           return np.array([xGoal, (((xGoal-self.ballPos[0])/self.ballVel[0])*self.ballVel[1])+self.ballPos[1]])
        #Se não acompanha o y
        return np.array([xGoal, self.ballPos[1]])
"""
DEFINE ROBOTS 
ori = dot(norm(v_robot) , norm((pos_ball - pos_robot)))
dist = abs(CG - pos_robot) 

CG - centro do gol

"""