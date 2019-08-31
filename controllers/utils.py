import numpy as np

from communication.msg import comm_msg


def convSpeeds2Motors(velocities):
    norm_velocities = comm_msg()
    convertion = (512*19) / 100
    wheel_reduction = 3/1
    r = 0.03
    L = 0.075

    max_tics_per_s = 70000.
    encoder_resolution = 512.*19
    max_motor_speed = (max_tics_per_s) / encoder_resolution

    for i in range(3):
        lin_wR = (velocities.linear_vel[i] + (L/2)*velocities.angular_vel[i]) / r
        lin_wL = (velocities.linear_vel[i] - (L/2)*velocities.angular_vel[i]) / r

        wR = lin_wR/(np.pi*2) # Rot/s
        wL = lin_wL/(np.pi*2) # Rot/s

        wR = wR * wheel_reduction * convertion
        wL = wL * wheel_reduction * convertion

        
        if np.fabs(wR) > max_tics_per_s/100 or np.fabs(wL) > max_tics_per_s/100:
            if np.fabs(wR) >= np.fabs(wL):
                wL = max_tics_per_s/100 * wL/np.fabs(wR)
                wR = max_tics_per_s/100 * wR/np.fabs(wR)
            elif np.fabs(wL) >= np.fabs(wR):
                wR = max_tics_per_s/100 * wR/np.fabs(wL)
                wL = max_tics_per_s/100 * wL/np.fabs(wL)

        norm_velocities.MotorA[i] = int(wL)
        norm_velocities.MotorB[i] = int(wR)

        
    return norm_velocities