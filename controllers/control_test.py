import control
import numpy as np

x_r = None
y_r = None
th_r = None
u1_r = None
u2_r = None

x = 0
y = 0
th = 0
u1 = 0
u2 = 0


def update_state():
    global dx
    global dy
    global dth

    dx = x - x_r
    dy = y - y_r
    dth = th - th_r


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

clsd_loop_poles = [-5, -5, -25]

K = control.place(A, B, clsd_loop_poles)

state_vector = [[dx],
                [dy],
                [dth]]

saida = []
