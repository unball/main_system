#!/usr/bin/env python
import control
import numpy as np

import rospy

from vision.msg import VisionMessage
from communication.msg import robots_speeds_msg


def update(data, state_vector):
    state_vector[0] = data.x[0]/10 - x_r
    state_vector[1] = data.y[0]/10 - y_r
    state_vector[2] = data.th[0] - th_r


def start_system():
    r = 0.03
    R = 0.075/2
    global x_r
    global y_r
    global th_r

    x_r = 0.0
    y_r = 0.0
    th_r = 0

    u1_r = 1
    u2_r = 1

    state_vector = [[0],
                    [0],
                    [0]]

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
    clsd_loop_poles = [-1, -1, -5]

    rospy.init_node('control_system')
    rospy.Subscriber('vision_output_topic', VisionMessage, update, state_vector)
    pub = rospy.Publisher('robots_speeds', robots_speeds_msg, queue_size=1)
    rate = rospy.Rate(30)

    while True:
        A[0][2] = -1*np.sin(th_r)*u1_r

        B[0][1] = np.cos(th_r)
        B[1][1] = np.sin(th_r)

        K = control.place(A, B, clsd_loop_poles)

        # output = np.dot(C, state_vector)
        du = np.dot(-K, state_vector)

        u1 = du[0] + u1_r
        u2 = du[1] + u2_r


        print("du: ",[u1, u2])
        print("K: ", K)
        print("state_vector: ", state_vector)
        print("----------------")

        msg = robots_speeds_msg()
        msg.linear_vel(0) = u1
        msg.angular_vel(0) = u2

        # msg = comm_msg()
        # msg.MotorB[0] = 1/r*u1 - (R/r)*u2
        # msg.MotorA[0] = 1/r*u1 + (R/r)*u2

        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    print("---------- TESTING CONTROL ----------")
    start_system()
