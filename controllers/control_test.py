import control
import numpy as np
import matplotlib as plt


def update_state():
    global dx
    global dy
    global dth
    global du1
    global du2
    global state_vector

    dx = x - x_r
    dy = y - y_r
    dth = th - th_r

    state_vector = [[dx],
                    [dy],
                    [dth]]


x_r = 1
y_r = 1
th_r = np.pi
u1_r = 1
u2_r = 1

x = 0
y = 0
th = 0
u1 = 0
u2 = 0

A = [[0, 0, -1*np.sin(th_r)*u1_r],
     [0, 0, np.cos(th_r)*u1_r],
     [0, 0, 0]]

B = [[np.cos(th_r), 0],
     [np.sin(th_r), 0],
     [0, 1]]

C = [[1, 0, 0],
     [0, 1, 0],
     [0, 0, 1]]

D = [[0, 0, 0],
     [0, 0, 0],
     [0, 0, 0]]


global clsd_loop_poles
clsd_loop_poles = [-5, -5, -25]

for t in range(5):
    update_state()

    A[0][2] = -1*np.sin(th_r)*u1_r

    B[0][1] = np.cos(th_r)
    B[1][1] = np.sin(th_r)

    K = control.place(A, B, clsd_loop_poles)

    du = -K * state_vector
    output = C * state_vector