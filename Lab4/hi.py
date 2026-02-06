from motorgo import ControlMode
import time

left.control_mode = ControlMode.POWER
right.control_mode = ControlMode.POWER

k_p = 15.0
k_d = 0.8

prev_error = 0.0
prev_t = time.time()

while True:
    # timing
    t = time.time()
    dt = t - prev_t
    prev_t = t
    if dt <= 0:
        continue

    # sensor
    g_theta = imu.gravity_vector[1]   # tilt estimate

    # PD
    error = -g_theta                  # target = 0
    d_error = (error - prev_error) / dt
    prev_error = error

    u = k_p * error + k_d * d_error

    # saturate
    u = max(min(u, 1.0), -1.0)

    # apply
    left.power_command = u
    right.power_command = u

    time.sleep(0.005)   # ~200 Hz
