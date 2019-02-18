#!/usr/bin/env python
import control
import numpy as np

import rospy
from cv2 import getTickCount
from cv2 import getTickFrequency

from vision.msg import VisionMessage
from python_simulator.msg import comm_msg
# from communication.msg import robots_speeds_msg

linear_saturation = 5
angular_saturation = 100

k_linear = 3
k_angular = 4


def update(data, state_vector):
    state_vector[0] = data.x[0]/100 - x_r
    state_vector[1] = data.y[0]/100 - y_r
    state_vector[2] = data.th[0] - th_r
    # state_vector[2] = np.linalg.norm(data.th[0]) - np.linalg.norm(0)


def start_system():
    e1 = getTickCount()
    time = 0
    r = 0.03
    R = 0.075/2
    global x_r
    global y_r
    global th_r

    x_r = 0.5
    y_r = -0.5
    th_r = 0

    u1_r = 1
    u2_r = 1

    state_vector = [[1],
                    [1],
                    [1]]

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
    clsd_loop_poles = [-2, -2, -5]

    rospy.init_node('control_system')
    rospy.Subscriber('vision_output_topic', VisionMessage, update, state_vector)
    pub = rospy.Publisher('robots_speeds', comm_msg, queue_size=1)
    rate = rospy.Rate(30)

    while True:
        e2 = getTickCount()

        A[0][2] = -1*np.sin(th_r)*u1_r
        A[1][2] = np.cos(th_r)*u1_r

        B[0][0] = np.cos(th_r)
        B[1][0] = np.sin(th_r)

        K = control.place(A, B, clsd_loop_poles)

        d_state_vector = ((A - np.dot(B, K)), state_vector)

        output = np.dot(C, state_vector)
        print(output)
        # print(u1_r, u2_r)
        du = np.dot(-K, state_vector)

        u1 = du[0]
        u2 = du[1]

        u1 = u1 + u1_r
        u2 = u2 + u2_r

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

        # print("du: ", [u1, u2])
        # print("K: ", K)
        # print("state_vector: ", state_vector)
        # print("----------------")

        # msg = robots_speeds_msg()
        # msg.linear_vel(0) = u1
        # msg.angular_vel(0) = u2

        msg = comm_msg()
        msg.MotorB[0] = 1/r*u1 - (R/r)*u2
        msg.MotorA[0] = 1/r*u1 + (R/r)*u2

        pub.publish(msg)
        rate.sleep()

        # x_r = 0.5*np.sin(time/10)
        # y_r = 0.5*np.sin(time/20)
        # th_r = np.arctan2(y_r, x_r)

        u1_r = k_linear * np.sqrt(np.power(state_vector[0], 2) + np.power(state_vector[1], 2))
        u2_r = k_angular * state_vector[2]

        time = (e2 - e1)/getTickFrequency()
        # print(time)


if __name__ == '__main__':
    print("---------- TESTING CONTROL ----------")
    start_system()
