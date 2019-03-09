#!/usr/bin/env python
"""Control system module."""
import numpy as np
import control

from communication.msg import robots_speeds_msg
from communication.msg import comm_msg

ref_lin_vel = 1
ref_ang_vel = 1


class ssRegulator():
    """Class docstring."""

    def __init__(self):
        """Init method."""
        self.output_vel = robots_speeds_msg()
        self.number_of_robots = 3

        self.x_i = list(0 for i in range(self.number_of_robots))
        self.y_i = list(0 for i in range(self.number_of_robots))
        self.th_i = list(0 for i in range(self.number_of_robots))

        self.x_r = list(0 for i in range(self.number_of_robots))
        self.y_r = list(0 for i in range(self.number_of_robots))
        self.th_r = list(0 for i in range(self.number_of_robots))

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
        self.poles = [[-1, -1, -4],
                      [-1, -1, -3],
                      [-1, -1, -3]]

    def actuate(self, references, world):
        """Control system actuator itself. Receives references and world info."""
        self.updateIntVariables(references, world)
        self.updateDynamicMatrices()
        self.updateStateVector()
        self.controlLaw()
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
                if np.linalg.norm(self.th_e[i]) > np.pi/2:
                    self.th_e[i] = (self.th_e[i] + np.pi/2) % (np.pi) - np.pi/2
                    self.x_e[i] = (-1) * self.x_e[i]
                    self.y_e[i] = (-1) * self.y_e[i]

                self.state_vector[i] = [self.x_e[i], self.y_e[i], self.th_e[i]]

    def controlLaw(self):
        for i in range(self.number_of_robots):
            # K, S, E = control.lqr(self.A[i], self.B[i], self.Q, self.R)
            K = control.place(self.A[i], self.B[i], self.poles[i])
            velocities = np.dot(-K, self.state_vector[i])
            self.output_vel.linear_vel[i] = velocities[0] # + np.sign(velocities[0])*0.5
            self.output_vel.angular_vel[i] = velocities[1]
