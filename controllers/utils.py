import numpy as np

def convSpeeds2Motors(velocities):
    convertion = (512*19) / 100
    wheel_reduction = 3/1
    r = 0.03
    L = 0.075

    max_tics_per_s = 70000.
    encoder_resolution = 512.*19
    max_motor_speed = (max_tics_per_s) / encoder_resolution

    for i in range(3):
        lin_wR = (velocities.linear_vel[i] + (L/2)*velocities.linear_vel[i]) / r
        lin_wL = (velocities.linear_vel[i] - (L/2)*velocities.linear_vel[i]) / r

        wR = lin_wR/(np.pi*2) # Rot/s
        wL = lin_wL/(np.pi*2) # Rot/s

        wR = wR * wheel_reduction * convertion
        wL = wL * wheel_reduction * convertion

        if np.fabs(wR) > max_motor_speed or np.fabs(wL) > max_motor_speed:
            if np.fabs(wR) >= np.fabs(wL):
                wL = max_motor_speed * wL/np.fabs(wR)
                wR = max_motor_speed * wR/np.fabs(wR)
            elif np.fabs(wL) >= np.fabs(wR):
                wR = max_motor_speed * wR/np.fabs(wL)
                wL = max_motor_speed * wL/np.fabs(wL)


        return wR, wL