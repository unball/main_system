"""Attack decider module."""

from strategy.secondleveldeciders.secondleveldecider import SecondLvlDecider

from strategy.players.striker import Striker
from strategy.players.goalkeeper import Goalkeeper
from strategy.players.defender import Defender

from strategy.membership_functions import *
import numpy as np
from strategy.megafunctions import fuzzy
from strategy.megafunctions import defuzzy
from math import sin
from random import randint
from math import acos
import matplotlib.pyplot as plt

magic_number = 14
LEFT = -1
RIGHT = 1
allFormationsS = ['GDD', 'GDS', 'GSS', 'DSS']
allFormations = [[Goalkeeper(), Defender(), Defender()], \
                [Goalkeeper(), Defender(), Striker()], \
                [Goalkeeper(), Striker(), Striker()], \
                [Defender(), Striker(), Striker()]]
#plt.ion() ## Note this correction
#fig=plt.figure()
#plt.ylim((-.6, .6))   # set the ylim to bottom, top
#plt.xlim(-.75, .75)     # set the ylim to bottom, to


class AttackDecider(SecondLvlDecider):
    """Class docstring."""

    def __init__(self):
        self.pos = np.matrix([[.0,0],[0,0],[0,0]])
        self.vel = np.matrix([[.0,0],[0,0],[0,0]])
        self.th = np.matrix([[.0],[.0],[.0]])
        self.ballPos = np.array([.0, .0])
        self.ballVel = np.array([.0, .0])
        self.targets = np.matrix([[.0,0],[0,0],[0,0]])
        self.per_robot = [-1.0, -0, 1.0]
        self.formationS = "GDS"
        self.formation = [Goalkeeper(), Defender(), Striker()]
        self.game_score = .0
        self.finalTarget = np.array([.75, 0])
        self.bounds = np.array([-.25, .25])
        self.fieldSide = 1


    #1 seta os parâmetros iniciais
    def setParams(self, world):
        self.definePosVel(world)
        self.defineTarget(world.fieldSide)
        self.defineBounds(world.gameScore)
    
    #2 retorna a formação
    def setFormation(self, world):
        self.idFormation(world.gameScore)
        self.rearrange_formation(world)

    #identifica a melhor formação
    def idFormation(self, gameScore):

        #FAM é a matrix de especialista
        FAM = np.matrix([[0, 1, 3],[0, 1, 2],[0, 1, 2], [0, 2, 2],[2, 3, 3]])

        boundMin = min(self.bounds)
        boundMax = max(self.bounds)
        if self.fieldSide == RIGHT:
            boundMin = max(self.bounds)
            boundMax = min(self.bounds)
        topDist = np.array([boundMin, ((boundMin+boundMax)/2), boundMax])

        topScore = np.array([-7, -3, 0, 3, 7])

        fuzzyDist = fuzzy(self.ballPos[0],topDist[:-1],topDist, topDist[1:])
        fuzzyScore = fuzzy(gameScore,topScore[:-1],topScore, topScore[1:])

        self.formationS = allFormationsS[FAM[fuzzyScore.argmax(), fuzzyDist.argmax()]]
        self.formation = allFormations[FAM[fuzzyScore.argmax(), fuzzyDist.argmax()]]

    def maxi(self,per, jf):
        maxval  = -2.0
        maxi = 0
        for i in range(0,3):
            if per[i]>maxval and jf[i]!=-1:
                maxval =per[i]
                maxi = i
        return maxi

    #rearranja a lista de formação para a troca de função entre os robôs
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
        for i in range(3):
            self.vel[i][np.isnan(self.vel[i, 0])]  = np.cos(self.th[i])
            self.vel[i][np.isnan(self.vel[i, 1])]  = np.sin(self.th[i])
        # self.vel[np.isnan(self.vel)] = 0
        
        """orientacao pra fuzzy ori"""
        ori = np.multiply(self.vel,dist_BR).sum(1)
        
        """Distancia pro centro do nosso gol-robo (harcoded) - MUDAR"""
        dist_CG = np.array([.75*self.fieldSide,.0]) - self.pos
        dist_CG = np.sqrt(np.power(dist_CG,2).sum(1))
        #print(dist_CG)
        """per_robot pertinencia ao ataque de cada robo (-1,+1)"""
        alpha = .15 
        perPrevius = np.array(self.per_robot.copy())
        perPrevius[np.isnan(perPrevius)] = .0
        
        for i in range(0,3):
            fuzzy_dist = fuzzy(dist_CG[i,0],tops_dist[:-1],tops_dist, tops_dist[1:])
            fuzzy_ori = fuzzy(ori[i,0],tops_ori[:-1],tops_ori, tops_ori[1:])
            self.per_robot[i] = alpha*(defuzzy(fuzzy_ori,FAM,fuzzy_dist)) + (1-alpha)* perPrevius[i]
        """rearranjando formação..."""
        self.per_robot[0] = -10
        new_formation = [Defender(),Defender(),Defender()]
        seek = [Striker(), Defender(), Goalkeeper()]
        ja_foi = [0, 0, 0]
        jfnf = [0,0,0]              #ja foi nao foi
        for k in range(0, 3):
            for i in range(0, 3):
                if (seek[k].id == self.formation[i].id and ja_foi[i] != -1):
                    maxi = self.maxi(self.per_robot.copy(),  jfnf)
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
        mirrorCenter = np.array([-.375*self.fieldSide, .0]) #centro da simetria radial
        radiusMin = .1                      #raio minimo para simetria
        rectangle = np.array([[.1,.4], [.55, -.4]])
        targetMid = np.array([0, 0])
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
                if self.fieldSide == RIGHT:
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
    def blockBallElipse(self):
        ########definir z = [a**2, b**2]###########
        z = np.array([.4**2, .2**2])
        Goal = np.array([-.7, 0])
        if (self.ballVel[0]*self.fieldSide) > .15:
            c =  [sum(z*(self.ballVel)**2), 2*np.dot(z*self.ballVel, self.ballPos-Goal), sum(z*(self.ballPos-Goal)**2) - np.prod(z)]
            k = min(np.roots(c))
            if k.imag == 0:
                target = k*self.ballVel + self.ballPos
                if(not self.ballInsideArea()):
                    return target
                target[1] = target[1] + .1
                return target
        #apenas fica entre a bola e o Goal
        #if not self.ballInsideArea():
        if sum(z*(self.ballPos-Goal)) > 0:
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
        alpha = .1
        targetsPrevius = self.targets.copy()
        perRobot = np.array(self.per_robot.copy())
        perRobot[0] = -10
        argMax, argMid, argMin = self.robotArgs(perRobot)
        #shooter
        self.targets[argMax] = self.shoot(argMax) #filtrar possibilidade de estar na area
        #self.targets[argMax] = self.ballPos
        #print(self.targets[argMax])
        #assistente e defensor
        
        attack = self.pair_attack()
        if attack and ((self.formationS == "GSS") or (self.formationS == "DSS")):
            self.targets[argMid] = self.mirrorPos(argMax, argMid)
        elif attack and ((self.formationS == "GDS") or (self.formationS == "GDD")):
            self.targets[argMid] = self.midFielder(argMax)
        else:
            #self.targets[argMid] = np.array([-0.5, self.ballPos[1]])
            self.targets[argMid] = self.blockBallElipse()

        #goleiro e ultimo defensor
        if self.formationS == "GSS" or \
           self.formationS == "GDS" or \
           self.formationS == "GDD" or \
           self.ballPos[0]*self.fieldSide > .20:
           self.targets[argMin] = self.goalkeep()
        else:
            #self.targets[argMid] = np.array([-0.5, self.ballPos[1]])
            self.targets[argMin] = self.blockBallElipse()
        
        self.avoidance()
        self.filtring(argMax,argMid,argMin)
        self.targets = alpha*self.targets + (1-alpha)*targetsPrevius

        #plt.scatter(self.targets[argMax][0,0],self.targets[argMax][0,1])
        #plt.pause(.03)

        # print(self.targets)
        # print(argMax,argMid,argMin)
        # print('----------')

        angles = np.arctan2((self.targets - self.pos)[:,1], (self.targets - self.pos)[:,0]).tolist()
        targets = self.targets.tolist()
        for i in range(len(targets)):
            targets[i].append(angles[i][0])
        return targets
        
    #calcula o target robô levar a bola ao gol
    def shoot(self, shooter):
        robotBall = np.array(self.ballPos - self.pos[shooter])[0]  
        ballTarget = self.finalTarget - np.array(self.ballPos)[0]
        dot = np.dot(robotBall, ballTarget)
        normrobotBall = np.linalg.norm(robotBall)
        normballTarget = np.linalg.norm(ballTarget) 
        fi = 2
        if normrobotBall*normballTarget!=0:
            fi = acos(dot/(normrobotBall*normballTarget))
        #else:
        #    return self.finalTarget
        distEball = .3*sin(fi) 
        if fi < np.pi/9:
            return self.ballPos
            #return self.ballPos
        #elif normrobotBall < .05 :
        #    return self.finalTarget

        r1 = normballTarget + abs(distEball) + 0.05
        xe = distEball*self.finalTarget[0]+ r1*self.ballPos[0]/(r1+distEball)
        ye = distEball*self.finalTarget[1]+ r1*self.ballPos[1]/(r1+distEball)
        return np.array([xe,ye])

    def shoot2(self, shooter):
        robotBall = np.array(self.ballPos - self.pos[shooter])[0]  
        ballTarget = self.finalTarget - np.array(self.ballPos)[0]
        dot = np.dot(robotBall, ballTarget)
        normrobotBall = np.linalg.norm(robotBall)
        normballTarget = np.linalg.norm(ballTarget) 
        cos_fi = 1
        if normrobotBall*normballTarget!=0:
            cos_fi = dot/(normrobotBall*normballTarget)
        return self.finalTarget + (1-.3*(1+np.linalg.norm(robotBall))*cos_fi)*(self.ballPos - self.finalTarget)

    def shoot3(self, shooter):
        robotBall = np.array(self.ballPos - self.pos[shooter])[0]  
        ballTarget = self.finalTarget - np.array(self.ballPos)[0]
        m2 = robotBall[1]/robotBall[0]
        m =  ballTarget[1]/ ballTarget[0]
        fi = np.arctan2((m2-m),(1+m*m2))
        if abs(fi)>3.1415/9: 
            x = self.ballPos[0] + .35*self.fieldSide
            return np.array([x, m*(x-self.ballPos[0]) + self.ballPos[1]])
        return self.finalTarget
            
    #define o target final (no gol adiversario)
    def defineTarget(self, fieldSide):
        self.fieldSide = fieldSide
        if self.ballVel[0] < (.005*fieldSide): 
            self.finalTarget =  np.array([-.75*fieldSide, 0])

    def defineBounds(self, gameScore):
        self.bounds =  (self.fieldSide*(gameScore * .025)) +  np.array([-.25,.25])

    def definePosVel(self, world):
        robots = world.robots
        ball = world.ball
        for i in range(0,3):
            self.pos[i] = np.array(robots[i].pos)
            self.vel[i] = np.array(robots[i].vel)
            self.th[i] = robots[i].th
        self.ballPos = np.array(ball.pos)
        self.ballVel = np.array(ball.vel)

    def goalkeep(self):
        xGoal = self.fieldSide * .75
        #testar velocidade minima (=.15?)
        if ((self.ballVel[0]*self.fieldSide) > .1) and \
           ((self.ballPos[0]*self.fieldSide)> .0):
           #verificar se a projeção está no gol
           #projetando vetor até um xGoal-> y = (xGoal-Xball) * Vyball/Vxball + yBall
           y =  (((xGoal-self.ballPos[0])/self.ballVel[0])*self.ballVel[1])+self.ballPos[1]
           return np.array([xGoal-.05*self.fieldSide, max(min(.23, y),-.23)])
        #Se não acompanha o y
        return np.array([xGoal-.05*self.fieldSide, max(min(self.ballPos[1],.23),-.23)])

    def avoidance(self):
        perRobot = np.array(self.per_robot.copy())
        argMax, argMid, argMin = self.robotArgs(perRobot)

        # xGoal = self.fieldSide * .75
        # y =  (((xGoal-self.ballPos[0])/self.ballVel[0])*self.ballVel[1])+self.ballPos[1]
        # RobotMinBall = np.array([xGoal-.05*self.fieldSide, max(min(.23, y),-.23)]) - self.pos[argMin]
        # RobotMinBallNorm = np.linalg.norm(RobotMinBall)
        #se mid está indo a caminho do max-> desvia
        # if self.ballVel[0]*self.fieldSide > .1 and self.ballPos[0]*self.fieldSide > 0 and RobotMinBallNorm!=0 :
        #    goalk = [ self.targets[argMin,0],  (RobotMinBall[0,1])/ RobotMinBallNorm**3  + self.targets[argMin,1] ]
        #    self.targets[argMin] = [goalk[0], max(min(goalk[1], .3), -.3)]


        
        # robotTarget = self.targets[argMax] - self.pos[argMax]
        # robotTargetNorm = robotTarget / np.linalg.norm(robotTarget)
        # RobotMaxBall = self.ballPos - self.pos[argMax]
        # RobotMaxBallNorm = np.linalg.norm(RobotMaxBall)
        # # se max está indo a caminho da bola
        # if  self.vel[argMax,0]*self.fieldSide > 0:
        #     self.targets[argMax] =  self.ballPos + np.dot(robotTargetNorm, (RobotMaxBall/RobotMaxBallNorm).transpose()) *(.01*np.array(-RobotMaxBall[0,1], RobotMaxBall[0,0])/RobotMaxBallNorm**2)
        robotTarget = self.targets[argMax] - self.pos[argMax]
        robotTargetNorm = robotTarget / np.linalg.norm(robotTarget)
        RobotMaxMin = self.pos[argMin] - self.pos[argMax]
        RobotMaxMinNorm = np.linalg.norm(RobotMaxMin)
        # se max está indo a caminho do min  desvia
        if RobotMaxMinNorm!=0:
            self.targets[argMax] = -.01*(RobotMaxMin/ (RobotMaxMinNorm)**3) + self.targets[argMax]
        #if np.dot(robotTargetNorm, (RobotMaxMin/RobotMaxMinNorm).transpose()) > .7:
        #    self.targets[argMax] =  self.pos[argMin] + .01*np.array(-RobotMaxMin[0,1], RobotMaxMin[0,0])/RobotMaxMinNorm**2
        robotTarget = self.targets[argMid] - self.pos[argMid]
        robotTargetNorm = robotTarget / np.linalg.norm(robotTarget)
        RobotMidMax = self.pos[argMax] - self.pos[argMid]
        RobotMidMaxNorm = np.linalg.norm(RobotMidMax)
        # se mid está indo a caminho do max-> desvia
        if RobotMidMaxNorm!=0:
            self.targets[argMid] = -.01*(RobotMidMax/ (RobotMidMaxNorm)**3) + self.targets[argMid]
        #if np.dot(robotTargetNorm, (RobotMidMax/RobotMidMaxNorm).transpose()) > .7: 
        #    self.targets[argMid] =  self.pos[argMax] + .01*np.array(-RobotMidMax[0,1], RobotMidMax[0,1])/RobotMidMaxNorm**2
        robotTarget = self.targets[argMid] - self.pos[argMid]
        robotTargetNorm = robotTarget / np.linalg.norm(robotTarget)
        RobotMidMin = self.pos[argMin] - self.pos[argMid]
        RobotMidMinNorm = np.linalg.norm(RobotMidMin)
        # se mid está indo a caminho do min-> desvia
        if RobotMidMaxNorm!=0:
            self.targets[argMid] = -.01*(RobotMidMin/ (RobotMidMinNorm)**3) + self.targets[argMid]
        #if np.dot(robotTargetNorm, (RobotMidMin/RobotMidMinNorm).transpose()) > .7:
        #    self.targets[argMid] =  self.pos[argMin] + .01*np.array(-RobotMidMin[0,1], RobotMidMin[0,0])/RobotMidMinNorm**2    
       
        """
        xv = -(+/-)*.01*Yt/(xt^2+yt^2)
        yv = (+/-)*.01*xt/(xt^2+yt^2)
        target = desviado + v
        """

    def filtring(self,argMax,argMid, argMin):
        self.targets[argMax] =  [max(min(self.targets[argMax,0],.75),-.75), max(min(self.targets[argMax,1],.65),-.65)]
        if abs(self.targets[argMax,1])<.35 and self.targets[argMax,0]*self.fieldSide > .6 : #inside area
            self.targets[argMax,0] = .55*self.fieldSide 
        self.targets[argMid] =  [max(min(self.targets[argMid,0],.75),-.75), max(min(self.targets[argMid,1],.65),-.65)]
        if abs(self.targets[argMid,1])<.35 and self.targets[argMid,0]*self.fieldSide > .6 : #inside area
            self.targets[argMid,0] = .55*self.fieldSide
        self.targets[argMin] =  [max(min(self.targets[argMin,0],.75),-.75), max(min(self.targets[argMin,1],.65),-.65)]

"""
DEFINE ROBOTS  
ori = dot(norm(v_robot) , norm((pos_ball - pos_robot)))
dist = abs(CG - pos_robot) 

CG - centro do gol

"""
