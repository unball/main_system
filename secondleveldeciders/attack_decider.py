"""Attack decider module."""

from secondleveldeciders.secondleveldecider import SecondLvlDecider

from players.striker import Striker
from players.goalkeeper import Goalkeeper
from players.defender import Defender

from membership_functions import *
import numpy as np
from megafunctions import fuzzy
from megafunctions import defuzzy

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
        self.target = np.array([.0, .0])
        self.targets = np.matrix([[.0,0],[0,0],[0,0]])
        self.per_robot = []
        self.rearranged_formation = np.array([])
        self.formation_S = "GDS"
        self.game_score =0

    def id_formation(self, game_score):
        """Identify the formation based on the world state."""
        score = self.FUZZYscore(0)#game_score
        self.game_score = game_score/10
        if type(score) == type([]):
            self.formation = score
            return None
        ball_x = magic_number * self.FUZZYball_x(self.world.ball.pos[0])
        total_score = ball_x + score
        print("score: ", score)
        self.formation, _ = self.defuzzicator(total_score)

    def FUZZYscore(self, value):
        """Docstring."""
        #super attack
        if value <= -7 or value >= 7:
            return [Defender(), Striker(), Striker()]
        elif value > -7 and value <= -5:
            FA = i_neg(-7, -5, value)
            D = i_pos(-7, -5, value)
            return (FA * (-7)) + (D * (-5))
        elif value > -5 and value <= 0:
            D = i_neg(-5, 0, value)
            N = i_pos(-5, 0, value)
            return (D * (-5)) + (N * (0))
        elif value > 0 and value <= 5:
            N = i_neg(0, 5, value)
            A = i_pos(0, 5, value)
            return (N * (0)) + (A * (5))
        elif value > 5 and value <= 7:
            A = i_neg(5, 7, value)
            FA = i_pos(5, 7, value)
            return (A * (5)) + (FA * (7))

    def FUZZYball_x(self, value):
        """Docstring."""
        if value < -0.5:
            return -0.5
        elif value > 0.5:
            return 0.5
        elif value > -0.5 and value <= -0.25:
            SD = i_neg(-0.5, -0.25, value)
            D = i_pos(-0.5, -0.25, value)
            return (SD * (-0.5)) + (D * (-0.25))
        elif value > -0.25 and value <= 0:
            D = i_neg(-0.25, 0, value)
            N = i_pos(-0.25, 0, value)
            return (D * (-0.25)) + (N * (0))
        elif value > 0 and value <= 0.25:
            N = i_neg(0, 0.25, value)
            A = i_pos(0, 0.25, value)
            return (N * (0)) + (A * (0.25))
        elif value > 0.25 and value <= 0.5:
            A = i_neg(0.25, 0.5, value)
            SA = i_pos(0.25, 0.5, value)
            return (A * (0.25)) + (SA * (0.5))

    def defuzzicator(self, score):
        """Docstring."""
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

    def rearrange_formation(self,world):
        """Rearrange the list of players."""
        """FAM é a matrix de especialista """
        FAM = np.matrix([[-1, -.9, -.2, 0, .4],[-.6,-.2,0,.6,.8],[.2,.4,.8,.9,1]])
        """tops_dist sao os locais de tops da megafunction dist """
        tops_dist = np.array([.35,.50,.85,1.20,1.35])
        """tops_dist sao os locais de tops da megafunction ori """
        tops_ori = np.array([0, .5, 1])
        """ pos,vel e ball são as posições dos robos, velocidade dos robos e posição da bola respectivamente"""
        self.pos = np.matrix([[.0,0],[0,0],[0,0]])
        self.vel = np.matrix([[.0,0],[0,0],[0,0]])
        ball = np.array(world.ball.pos)
        for i in range(0,3):
            self.pos[i] = np.array(world.robots[i].pos)
            self.vel[i] = np.array(world.robots[i].vel)
        """Distancia bola-robo"""
        dist_BR = ball - self.pos
        abs_dist_BR = np.power(np.power(dist_BR,2).sum(1), .5)
        """Distancia bola-robo normalizada"""
        dist_BR = dist_BR/abs_dist_BR

        abs_vel = np.power(np.power(self.vel,2).sum(1),.5)
        """velocidade robos normalizada"""
        self.vel = self.vel/abs_vel
        """orientacao pra fuzzy ori"""
        ori = np.multiply(self.vel,dist_BR).sum(1)

        """Distancia pro centro do nosso gol-robo (harcoded) - MUDAR"""
        dist_CG = np.array([.75,.0]) - self.pos
        dist_CG = np.power(np.power(dist_CG,2).sum(1),.5)
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
        self.rearranged_formation = np.array(new_formation)
        return new_formation

    # se for ataque retorna true 
    def pair_attack(self):
        ball_vel_x = self.world.ball.vel[0]
        ball_x = self.world.ball.pos[0]
        if self.world.fieldSide == LEFT:
            bonds = - (self.game_score * .025) +  np.array([-.25,.25])
            if ball_x > bonds[1]:
                    return True #ataque
            elif  ball_x > bonds[0] and ball_x <= bonds[1]:
                if self.formation_S == "GDS" or self.formation_S == "GSS":
                    if ball_vel_x >= 0:
                        return True #ataque
                elif self.formation_S=="DSS":
                    return True #ataque
        else:
            bonds = (self.game_score * .025) +  np.array([-.25,.25])
            if ball_x < bonds[1]:
                    return True #ataque
            elif  ball_x > bonds[0] and ball_x <= bonds[1]:
                if self.formation_S == "GDS" or self.formation_S == "GSS":
                    if ball_vel_x >= 0:
                        return True #ataque
                elif self.formation_S=="DSS":
                    return True #ataque
        return False
        
    def shoot(self):
        pass

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
        mirrorCenter = np.array([37.5, 0])  #centro da simetria radial
        radiusMin = 10                      #raio minimo para simetria
        rectangle = np.array([[10.0,40.0], [55.0, -40.0]])
        if self.world.fieldSide == RIGHT:   #se o lado de defesa for o direito troca os valores
            mirrorCenter = np.array([-37.5, 0])
            rectangle = np.array([[-55.0, 40.0], [-10.0, -40.0]])
        centerDom = mirrorCenter - self.pos[argMax] #centro do espelho - posição do dominante
        #se o robô dominante estiver denforatro do raio minimo, usa a simetria
        if centerDom > radiusMin:
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

    def updateTargets(self):
        perRobot = np.array(self.per_robot)
        argMax, argMid, argMin = self.robotArgs(perRobot)
        #self.targets[argMax] = self.shoot()
        if self.pair_attack():
            self.targets[argMid] = self.mirrorPos(argMax, argMid)
    
"""
DEFINE ROBOTS 
ori = dot(norm(v_robot) , norm((pos_ball - pos_robot)))
dist = abs(CG - pos_robot) 

CG - centro do gol

"""