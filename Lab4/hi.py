from motorgo import ControlMode
from motorgo.plink import Plink
import time

LEFT_CH = 3
RIGHT_CH = 1

def main():

    plink = Plink()
    plink.connect()
    imu = plink.imu

    left = getattr(plink, f"channel{LEFT_CH}")
    right = getattr(plink, f"channel{RIGHT_CH}")

    left.control_mode = ControlMode.POWER
    right.control_mode = ControlMode.POWER

    k_p = 15.8
    k_d = 0.26

    prev_error = 0.0
    prev_t = time.time()
    back_bias = 1.5

    while True:
        # timing
        t = time.time()
        dt = t - prev_t
        prev_t = t
        if dt <= 0:
            continue

        # sensor
        g_theta = imu.gravity_vector[1]   # tilt estimate
        # print("gtheta =" + f"{g_theta}")

        # PD -0.025
        error = -0.024 -g_theta                  # target = 0
        d_error = (error - prev_error) / dt
        prev_error = error

        u = k_p * error + k_d * d_error

        # saturate
        u = max(min(u, 0.75), -0.75)
        # apply
        print(u)
        left.power_command = -u
        right.power_command = u

        time.sleep(0.005)   # ~200 Hz

if __name__ == "__main__":
    main()
