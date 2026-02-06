import time
import math
import numpy as np
from motorgo.plink import Plink, ControlMode

# ================= INITIALIZE =================
plink = Plink()
plink.connect()

left_motor = plink.channel1
right_motor = plink.channel3

left_motor.control_mode = ControlMode.POWER
right_motor.control_mode = ControlMode.POWER

left_motor.power_command = 0
right_motor.power_command = 0

# ========== ROBOT CONSTANTS ==========
WHEEL_RADIUS = 0.85      # inches
WHEELBASE = 7.6         # inches
TICKS_PER_REV = 6.698
DT = 0.01               # seconds
# ====================================

# ========== MOTOR DIRECTIONS ==========
LEFT_MOTOR_DIR = -1     # left motor physically flipped
RIGHT_MOTOR_DIR = 1
# =====================================

# ========== ENCODER SIGNS ==========
LEFT_ENC_SIGN = 1       # left encoder increases forward
RIGHT_ENC_SIGN = 1     # right encoder decreases forward
# ==================================

# ========== THREE MOTOR PAIRS ==========
# Format: (left_power, right_power)
MOTOR_PAIRS = [
    (-0.9, -0.72),
    (0.63, -0.4),
    (-0.55, 0.-7)
]
# ======================================


def rk4_step(state, v, omega):
    def f(s):
        return np.array([
            v * math.cos(s[2]),
            v * math.sin(s[2]),
            omega
        ])

    k1 = f(state)
    k2 = f(state + 0.5 * DT * k1)
    k3 = f(state + 0.5 * DT * k2)
    k4 = f(state + DT * k3)

    state = state + (DT / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    state[2] = math.atan2(math.sin(state[2]), math.cos(state[2]))
    return state


def update_odometry(state, prev_left, prev_right):
    curr_left = left_motor.position
    curr_right = right_motor.position

    delta_left = LEFT_ENC_SIGN * (curr_left - prev_left)
    delta_right = RIGHT_ENC_SIGN * (curr_right - prev_right)

    left_dist = (delta_left / TICKS_PER_REV) * (2 * math.pi * WHEEL_RADIUS)
    right_dist = (delta_right / TICKS_PER_REV) * (2 * math.pi * WHEEL_RADIUS)

    v_left = left_dist / DT
    v_right = right_dist / DT

    v = 0.5 * (v_left + v_right)
    omega = (v_right - v_left) / WHEELBASE

    state = rk4_step(state, v, omega)

    return state, curr_left, curr_right


def run_command(left_power, right_power, state, prev_left, prev_right, idx):
    print(f"\n--- Command {idx} ---")
    print(f"Motors: L={left_power}, R={right_power}")

    left_motor.power_command = LEFT_MOTOR_DIR * left_power
    right_motor.power_command = RIGHT_MOTOR_DIR * right_power

    start = time.time()
    while time.time() - start < 3.0:
        state, prev_left, prev_right = update_odometry(
            state, prev_left, prev_right
        )
        time.sleep(DT)

    left_motor.power_command = 0
    right_motor.power_command = 0

    x, y, theta = state
    print(f"x={x:.2f} in, y={y:.2f} in, θ={math.degrees(theta):.2f}°")

    return state, prev_left, prev_right


# ================= MAIN =================
if __name__ == "__main__":
    print("RK4 Odometry with 3 Motor Commands")
    input("Place robot at (0,0,0) facing +x. Press Enter...\n")

    state = np.array([0.0, 0.0, 0.0])
    prev_left = left_motor.position
    prev_right = right_motor.position

    try:
        for i, (lp, rp) in enumerate(MOTOR_PAIRS, start=1):
            state, prev_left, prev_right = run_command(
                lp, rp,
                state,
                prev_left,
                prev_right,
                i
            )
            time.sleep(0.5)

        print("\n" + "=" * 50)
        print(f"FINAL POSITION: x={state[0]:.2f} in, y={state[1]:.2f} in")
        print(f"({state[0]},{state[1]})")
        print("=" * 50)

    except KeyboardInterrupt:
        left_motor.power_command = 0
        right_motor.power_command = 0
        print("Stopped.")
