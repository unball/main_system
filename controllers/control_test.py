#!/usr/bin/env python
import control
import numpy as np

import rospy
from cv2 import getTickCount
from cv2 import getTickFrequency

from vision.msg import VisionMessage
from communication.msg import robots_speeds_msg

linear_saturation = 5
angular_saturation = 100

k_linear = 3
k_angular = 4


def CorrectLinVel(lin_vel, theta):
    vel_x = np.cos(theta)*lin_vel
    vel_y = np.sin(theta)*lin_vel

    true_vel = (np.cos(theta) * vel_x) + (np.cos(np.pi/2 - theta) * vel_y)
    lateral_vel = (np.cos(np.pi/2 - theta)*vel_x) + (np.cos(theta) * vel_y)
    print(lateral_vel)
    # print("Lin_vel: {}\nTrue_vel: {}".format(lin_vel, true_vel))
    return true_vel


def correctErrors(sensor_x, sensor_y, _sensor_th):
    x_error = sensor_x - x_r
    y_error = sensor_y - y_r
    th_error = _sensor_th - th_r

    if np.linalg.norm(th_error) > np.pi/2:
        th_error = (th_error + np.pi/2) % (np.pi) - np.pi/2
        x_error = (-1) * x_error
        y_error = (-1) * y_error

    state_vector = [x_error, y_error, th_error]
    return state_vector


def update(data, state_vector):
    inst_x = data.x[0]
    inst_y = data.y[0]
    inst_th = data.th[0]

    aux = correctErrors(data.x[0], data.y[0], data.th[0])

    state_vector[0] = aux[0]
    state_vector[1] = aux[1]
    state_vector[2] = aux[2]
    # state_vector[3][0] = float(state_vector[3][0] - state_vector[0]*loop_time)
    # state_vector[4][0] = float(state_vector[4][0] - state_vector[1]*loop_time)
    # state_vector[5][0] = float(state_vector[5][0] - state_vector[2]*loop_time)


def start_system():
    e1 = getTickCount()
    global loop_time
    loop_time = 0
    time = 0
    r = 0.03
    L = 0.075
    global x_r
    global y_r
    global th_r

    global inst_x
    global inst_y
    global inst_th

    global xi
    global yi
    global thi

    x_r = 0
    y_r = 0
    th_r = 0

    inst_x = 0
    inst_y = 0
    inst_th = 0

    u1_r = 1
    u2_r = 1

    state_vector = [[0.1],
                    [0.1],
                    [0.1]]

    A = [[0, 0, -1*np.sin(th_r)*u1_r],
         [0, 0, np.cos(th_r)*u1_r],
         [0, 0, 0]]

    Ai = [[0, 0, -1*np.sin(th_r)*u1_r, 0, 0, 0],
          [0, 0, np.cos(th_r)*u1_r, 0, 0, 0],
          [0, 0, 0, 0, 0, 0],
          [-1, 0, 0, 0, 0, 0],
          [0, -1, 0, 0, 0, 0],
          [0, 0, -1, 0, 0, 0]]

    B = [[np.cos(th_r), 0],
         [np.sin(th_r), 0],
         [0, 1]]

    Bi = [[np.cos(th_r), 0],
          [np.sin(th_r), 0],
          [0, 1],
          [0, 0],
          [0, 0],
          [0, 0]]

    C = [[1, 0, 0],
         [0, 1, 0],
         [0, 0, 1]]

    Ci = [[1, 0, 0, 0, 0, 0],
          [0, 1, 0, 0, 0, 0],
          [0, 0, 1, 0, 0, 0]]

    D = [[0, 0, 0],
         [0, 0, 0],
         [0, 0, 0]]

    max_dx = 0.01
    max_dy = 0.01
    max_dth = 0.01

    max_xi = 1
    max_yi = 1
    max_thi = 1

    q0 = 1/(max_dx**2)
    q1 = 1/(max_dy**2)
    q2 = 1/(max_dth**2)

    # Estados integrativos
    q3 = 1/(max_xi**2)
    q4 = 1/(max_yi**2)
    q5 = 1/(max_thi**2)

    Q = [[q0, 0, 0],
         [0, q1, 0],
         [0, 0, q2]]

    Qi = [[q0, 0, 0, 0, 0, 0],
          [0, q1, 0, 0, 0, 0],
          [0, 0, q2, 0, 0, 0],
          [0, 0, 0, q3, 0, 0],
          [0, 0, 0, 0, q4, 0],
          [0, 0, 0, 0, 0, q5]]

    max_du1 = 1
    max_du2 = 1

    rho = 50

    r0 = (1/(max_du1**2)) * rho
    r1 = (1/(max_du2**2)) * rho

    R = [[r0, 0],
         [0, r1]]

    global clsd_loop_poles
    clsd_loop_poles = [-2, -2, -5]

    rospy.init_node('control_system')
    rospy.Subscriber('vision_output_topic', VisionMessage, update, state_vector)
    pub = rospy.Publisher('robots_speeds', robots_speeds_msg, queue_size=1)
    rate = rospy.Rate(30)

    while True:
        e2 = getTickCount()

        A[0][2] = -1*np.sin(th_r)*u1_r
        A[1][2] = np.cos(th_r)*u1_r

        B[0][0] = np.cos(th_r)
        B[1][0] = np.sin(th_r)

        Ai[0][2] = -1*np.sin(th_r)*u1_r
        Ai[1][2] = np.cos(th_r)*u1_r

        Bi[0][0] = np.cos(th_r)
        Bi[1][0] = np.sin(th_r)

        K, S, E = control.lqr(A, B, Q, R)
        # Ki, Si, Ei = control.lqr(Ai, Bi, Qi, R)
        # K = control.place(A, B, clsd_loop_poles)

        du = np.dot(-K, state_vector)
        print(state_vector)

        u1 = du[0]
        u2 = du[1]

        # u1 = CorrectLinVel(du[0], inst_th)

        '''
        if u1 > linear_saturation:
            u1 = linear_saturation
        elif u1 < linear_saturation:
            u1 = -1 * linear_saturation

        if u1 > angular_saturation:
            u1 = angular_saturation
        elif u1 < angular_saturation:
            u1 = -1 * angular_saturation
        '''

        msg = robots_speeds_msg()
        msg.linear_vel[0] = u1
        msg.angular_vel[0] = u2

        e3 = getTickCount()
        loop_time = (e3 - e2)/getTickFrequency()
        # print(loop_time)

        pub.publish(msg)
        rate.sleep()

        # x_r = x_r + 0.01
        # y_r = np.sin(x_r) * 0.3
        # th_r = np.arctan2(-output[1], -output[0])

        time = (e2 - e1)/getTickFrequency()
        # print(time)


if __name__ == '__main__':
    print("---------- TESTING CONTROL ----------")
    start_system()
