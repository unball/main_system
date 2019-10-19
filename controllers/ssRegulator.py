#!/usr/bin/env python
"""Control system module."""
import numpy as np
import control
import gui.mainWindow
from states.system import System


ref_lin_vel = 1
ref_ang_vel = 1

class SpeedPair():
    def __init__(self):
        self.v = 0
        self.w = 0
    
    def __str__(self):
        return "{" + "v: {0}, w: {1}".format(self.v, self.w) + "}" 

class nonLinearControl(System):
    def __init__(self, parent):
        System.__init__(self, parent)

        self.number_of_robots = 3
        self.output_vel = [SpeedPair() for i in range(self.number_of_robots)]
        self.th_e_ant = [0 for i in range(self.number_of_robots)]
        self.th_e =     [0 for i in range(self.number_of_robots)]
        self.w_ant =    [0 for i in range(self.number_of_robots)]
        self.x_i = list(0 for i in range(self.number_of_robots))
        self.y_i = list(0 for i in range(self.number_of_robots))

        self.x_r = list(0 for i in range(self.number_of_robots))
        self.y_r = list(0 for i in range(self.number_of_robots))

        self.int =    [0 for i in range(self.number_of_robots)]
        self.w_k1 = 0.997
        self.w_k2 = 0.07

        self.v_k = 0.25
        self.v_max = 0.8
        self.v_offset = 0.35

        self.th_i = [0 for i in range(self.number_of_robots)]
        self.th_r = [0 for i in range(self.number_of_robots)]


    def updateIntVariables(self, references, world):
        for i in range(self.number_of_robots):
            self.th_i[i] = world.robots[i].th
            self.x_i[i] = world.robots[i].x
            self.y_i[i] = world.robots[i].y
            self.th_r[i] = references[i][2]
            self.x_r[i] = references[i][0]
            self.y_r[i] = references[i][1]

    def sat(self, x, amp):
        return max(min(x, amp), -amp)


    def updateError(self):
            self.th_e_ant = list((self.th_e[i] + (self.output_vel[i].w -self.sat(self.output_vel[i].w, 4*np.pi)) for i in range(self.number_of_robots)))
            self.x_e = list((self.x_r[i]-self.x_i[i]) for i in range(self.number_of_robots))
            self.y_e = list((self.y_r[i]-self.y_i[i]) for i in range(self.number_of_robots))
            self.w_ant = [ov.w for ov in self.output_vel]
            
            self.th_e = list((self.th_r[i] - self.th_i[i]) for i in range(self.number_of_robots))
            for i in range(self.number_of_robots):
                if np.linalg.norm(self.th_e[i]) > np.pi/2:
                    self.th_e[i] = (self.th_e[i] + np.pi/2) % (np.pi) - np.pi/2


    def actuate(self, references, world):
        #self.output_vel = [SpeedPair() for i in range(self.number_of_robots)]
        """Control system actuator itself. Receives references and world info."""
        self.updateIntVariables(references, world)
        self.updateError()
        self.controlLaw(world)
        # print(self.output_vel)
        return self.output_vel

    def controlLaw(self, world):
        for i in range(self.number_of_robots):
            factor = 1*(1-np.e**(-1.5*(world.robots[i].pathLength()-0)))
            #if i==0: print(world.robots[i].pathLength())
            self.output_vel[i].w = 4/3*(self.th_e[i] + 1.4*np.sin(self.th_e[i]))*np.cos(self.th_e[i]) #(self.th_e[i] -  .997 * self.th_e_ant[i]) + self.output_vel[i].w
            
            #self.int[i] = self.w_k2*( 0.03/2 *(self.th_e[i]+self.th_e_ant[i]) + self.int[i])
            self.output_vel[i].w = self.sat(self.output_vel[i].w, 4*np.pi)
            self.output_vel[i].v = 15/3* np.sqrt(self.x_e[i]**2+self.y_e[i]**2) * abs(np.cos(self.th_e[i])) *  world.robots[i].dir#min(self.v_k/(abs(self.output_vel[i].w)+0.01), self.v_max)*world.robots[i].dir
            #f i==0: print("erro: "+ str(np.sqrt(self.x_e[i]**2+self.y_e[i]**2)))
            #self.output_vel[i].v = (0.15/abs(self.output_vel[i].w))*  world.robots[i].dir#min(self.v_k/(abs(self.output_vel[i].w)+0.01), self.v_max)*world.robots[i].dir
            #self.output_vel[i].v = 0.1*world.robots[i].dir # min(self.v_k/(abs(np.sin( self.th_e[i]))+0.01), self.v_max)*world.robots[i].dir

            if i==0: print("v: {0}, w: {1}, th: {2}".format(self.output_vel[i].v, self.output_vel[i].w, self.th_e[i]))


class ssRegulator(System):
    """Class docstring."""

    def __init__(self, parent):
        """Init method."""
        System.__init__(self, parent)
        self.output_vel = [[[], []],
                           [[], []],
                           [[], []]]
        self.number_of_robots = 3
        self.output_vel = [SpeedPair() for i in range(self.number_of_robots)]

        self.x_i = list(0 for i in range(self.number_of_robots))
        self.y_i = list(0 for i in range(self.number_of_robots))
        self.th_i = list(0 for i in range(self.number_of_robots))

        self.x_r = list(0 for i in range(self.number_of_robots))
        self.y_r = list(0 for i in range(self.number_of_robots))
        self.th_r = list(0 for i in range(self.number_of_robots))
        
        #!TODO: Suportar N robôs
        self.state_vector = [[0, 0, 0],  # Robot 0 state vector
                             [0, 0, 0],  # Robot 1 state vector
                             [0, 0, 0]]  # Robot 2 state vector

        self.updateStateVector()

        self.v_r = list(ref_lin_vel for i in range(self.number_of_robots))
        self.w_r = list(ref_ang_vel for i in range(self.number_of_robots))

        # State Space representation of a linear system x_p = Ax + Bu
        #                                                 y = Cx + Du
        self.A = [[], [], []]
        self.B = [[], [], []]

        for i in range(self.number_of_robots):
            self.A[i] = [[0, 0, -1*np.sin(self.th_r[i])*self.v_r[i]],
                         [0, 0, np.cos(self.th_r[i])*self.v_r[i]],
                         [0, 0, 0]]

            self.B[i] = [[np.cos(self.th_r[i]), 0],
                         [np.sin(self.th_r[i]), 0],
                         [0, 1]]

        self.C = [[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]]

        self.D = [[0, 0, 0],
                  [0, 0, 0],
                  [0, 0, 0]]

        # Linear Quadratic Regulator matrices and parameters
        self.max_x_e = 0.01
        self.max_y_e = 0.01
        self.max_th_e = 0.01

        self.q0 = 1/(self.max_x_e**2)
        self.q1 = 1/(self.max_y_e**2)
        self.q2 = 1/(self.max_th_e**2)

        self.Q = [[self.q0, 0, 0],
                  [0, self.q1, 0],
                  [0, 0, self.q2]]

        self.rho = 5000

        self.r0 = 1 * self.rho
        self.r1 = 1 * self.rho

        self.R = [[self.r0, 0],
                  [0, self.r1]]

        # Pole placement regulator closed loop poles
        self.poles = [[-5, -5, -0.1],
                      [-0.5, -0.5, -0.6],
                      [-0.5, -0.5, -0.8]]

    def actuate(self, references, world):
        #self.output_vel = [SpeedPair() for i in range(self.number_of_robots)]
        """Control system actuator itself. Receives references and world info."""
        self.updateIntVariables(references, world)
        self.updateDynamicMatrices()
        self.updateStateVector()
        self.controlLaw(world)
        # print(self.output_vel)
        return self.output_vel

    def updateIntVariables(self, references, world):
        for i in range(self.number_of_robots):
            self.x_i[i] = world.robots[i].x
            self.y_i[i] = world.robots[i].y
            self.th_i[i] = world.robots[i].th

            self.x_r[i] = references[i][0]
            self.y_r[i] = references[i][1]
            self.th_r[i] = references[i][2]

    def updateDynamicMatrices(self):
        for i in range(self.number_of_robots):
            self.A[i][0][2] = -1*np.sin(self.th_r[i])*self.v_r[i]
            self.A[i][1][2] = np.cos(self.th_r[i])*self.v_r[i]

            self.B[i][0][0] = np.cos(self.th_r[i])
            self.B[i][1][0] = np.sin(self.th_r[i])

    def updateStateVector(self):
            self.x_e = list((self.x_i[i] - self.x_r[i]) for i in range(self.number_of_robots))
            self.y_e = list((self.y_i[i] - self.y_r[i]) for i in range(self.number_of_robots))
            self.th_e = list((self.th_i[i] - self.th_r[i]) for i in range(self.number_of_robots))

            for i in range(self.number_of_robots):
                if np.linalg.norm(self.th_e[i]) > np.pi:
                    self.th_e[i] = (self.th_e[i] + np.pi) % (2*np.pi) - np.pi
                    self.x_e[i] = (-1) * self.x_e[i]
                    self.y_e[i] = (-1) * self.y_e[i]

                self.state_vector[i] = [self.x_e[i], self.y_e[i], self.th_e[i]]
                #print(self.state_vector)

    def controlLaw(self, world):
        import time
        dt = 0
        for i in range(self.number_of_robots):
            # K, S, E = control.lqr(self.A[i], self.B[i], self.Q, self.R)
            K = control.place(self.A[i], self.B[i], self.poles[i])
            velocities = np.dot(-K, self.state_vector[i])
            self.output_vel[i][0] = velocities[0]
            self.output_vel[i][1] = velocities[1]