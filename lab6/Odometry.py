import math
import time
import numpy as np

WHEEL_RADIUS = 1.125
WHEELBASE = 6.25       
TICKS_PER_REV = 6.4845
DT = 0.01                

LEFT_MOTOR_DIR = 1
RIGHT_MOTOR_DIR = -1

LEFT_ENC_SIGN = -1
RIGHT_ENC_SIGN = 1

def f(s, v, omega):
    return np.array([
        v * math.cos(s[2]),
        v * math.sin(s[2]),
        omega
    ])

def rk4_step(state, v, omega, dt):

    k1 = f(state, v, omega)
    k2 = f(state + 0.5 * dt * k1, v, omega)
    k3 = f(state + 0.5 * dt * k2, v, omega)
    k4 = f(state + dt * k3, v, omega)

    state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    return state


def update_odometry(state, prev_left, prev_right, left_motor, right_motor):
    curr_left = left_motor.position
    curr_right = right_motor.position

    delta_left = LEFT_ENC_SIGN * (curr_left - prev_left)
    delta_right = RIGHT_ENC_SIGN * (curr_right - prev_right)
    print(f"Encoders: L={curr_left:.2f} (Δ={delta_left:.2f}), R={curr_right:.2f} (Δ={delta_right:.2f})")

    dist_left = (delta_left / TICKS_PER_REV) * (2 * math.pi) * WHEEL_RADIUS
    dist_right = (delta_right / TICKS_PER_REV) * (2 * math.pi) * WHEEL_RADIUS

    # Linear and angular velocity
    v = 0.5 * (dist_left + dist_right) / DT
    omega = (dist_right - dist_left) / (WHEELBASE * DT)

    # Update full pose via RK4
    state = rk4_step(state, v, omega, DT)

    # Accumulate heading change for sector tracking
    delta_theta = (dist_right - dist_left) / WHEELBASE
    accumulated_angle += delta_theta

    return state, curr_left, curr_right, (accumulated_angle % 360)
