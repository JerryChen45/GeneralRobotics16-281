from motorgo import ControlMode
from motorgo.plink import Plink
import time

LEFT_CH = 3
RIGHT_CH = 1

def position(curr_pos, prev_pos, start_pos, dt, kp, kd, max_angle):
    pos_error = start_pos - curr_pos
    velocity = (curr_pos - prev_pos) / dt

    theta_target = kp * pos_error - kd * velocity
    return max(min(theta_target, max_angle), -max_angle)

def main():

    plink = Plink()
    plink.connect()
    imu = plink.imu

    left = getattr(plink, f"channel{LEFT_CH}")
    right = getattr(plink, f"channel{RIGHT_CH}")

    left.control_mode = ControlMode.POWER
    right.control_mode = ControlMode.POWER

    k_p = 7
    k_d = 0.28
    k_p_pos = 0.1
    k_d_vel = 0.1

    prev_error = 0.0
    prev_t = time.time()
    start_pos = (-left.position + right.position) / 2
    prev_pos = start_pos
    theta_target = 0
    max_angle = 0.05

    while True:
        # timing
        t = time.time()
        dt = t - prev_t
        dt = 0.1
        prev_t = t
        if dt <= 0:
            continue
        curr_pos = (-left.position + right.position) / 2
        theta_target = position(curr_pos, prev_pos, start_pos, dt, k_p_pos, k_d_vel, max_angle)
        prev_pos = curr_pos

        # sensor
        g_theta = imu.gravity_vector[1]   # tilt estimate
        # print("gtheta =" + f"{g_theta}")

        # PD -0.025
        error = theta_target - 0.024 - g_theta                  # target = 0
        d_error = (error - prev_error) / dt
        prev_error = error

        u = k_p * error + k_d * d_error

        # saturate
        u = max(min(u, 0.8), -0.9)
        # apply
        print(u)
        left.power_command = -u
        right.power_command = u

        time.sleep(0.05)   # ~200 Hz

if __name__ == "__main__":
    main()
