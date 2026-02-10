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

    k_p = 2.5
    k_d = 0.237
    k_i = 0.5

    prev_t = time.time()
    error_integral = 0.0
    integral_max = 0.3  # anti-windup clamp

    while True:
        # timing
        t = time.time()
        dt = t - prev_t
        prev_t = t
        if dt <= 0:
            continue

        # sensor
        g_theta = imu.gravity_vector[1]
        omega = imu.gyro[1]  # angular velocity from gyro

        # PID with gyro derivative
        error = -0.048 - g_theta
        error_integral += error * dt
        error_integral = max(min(error_integral, integral_max), -integral_max)

        u = k_p * error + k_i * error_integral - k_d * omega

        print(u)

        # saturate
        u = max(min(u, 0.6), -0.6)

        left.power_command = -u
        right.power_command = u

        time.sleep(0.005)

if __name__ == "__main__":
    main()