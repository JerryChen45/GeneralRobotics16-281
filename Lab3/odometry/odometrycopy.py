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
WHEEL_RADIUS = 1.0      # inches
WHEELBASE = 8.5         # inches
TICKS_PER_REV = 6.32   # <-- your calibrated value
DT = 0.01              # integration time step (seconds)
# ====================================

# ========== MOTOR DIRECTION ==========
LEFT_DIR = -1    # left motor is flipped
RIGHT_DIR = 1    # right motor is normal
# ====================================

# ========== COMMAND ==========
LEFT_POWER = 0.4
RIGHT_POWER = 0.6
# =============================


def update_odometry_rk4(state, prev_left, prev_right):
    """
    Update odometry using 4th-order Runge-Kutta (RK4) integration with numpy arrays.
    
    Args:
        state: numpy array [x, y, theta]
        prev_left: previous left encoder position
        prev_right: previous right encoder position
    
    Returns:
        state: updated numpy array [x, y, theta]
        curr_left: current left encoder position
        curr_right: current right encoder position
    """
    curr_left = left_motor.position
    curr_right = right_motor.position

    # Apply direction corrections to encoder deltas (motors are physically flipped)
    delta_left = (curr_left - prev_left) * LEFT_DIR
    delta_right = (curr_right - prev_right) * RIGHT_DIR

    # Convert encoder ticks to linear distances
    left_dist = (delta_left / TICKS_PER_REV) * (2 * math.pi * WHEEL_RADIUS)
    right_dist = (delta_right / TICKS_PER_REV) * (2 * math.pi * WHEEL_RADIUS)

    # Calculate linear and angular velocities
    v_left = left_dist / DT
    v_right = right_dist / DT
    
    v = (v_left + v_right) / 2.0        # linear velocity of robot center
    omega = (v_right - v_left) / WHEELBASE  # angular velocity

    # Define state derivative function
    def state_derivative(s):
        """Returns [dx/dt, dy/dt, dtheta/dt] given state s = [x, y, theta]"""
        return np.array([
            v * np.cos(s[2]),      # dx/dt
            v * np.sin(s[2]),      # dy/dt
            omega                   # dtheta/dt
        ])

    # RK4 Integration
    k1 = state_derivative(state)
    k2 = state_derivative(state + 0.5 * DT * k1)
    k3 = state_derivative(state + 0.5 * DT * k2)
    k4 = state_derivative(state + DT * k3)
    
    # Weighted average update
    state = state + (DT / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    
    # Normalize theta to [-pi, pi]
    state[2] = np.arctan2(np.sin(state[2]), np.cos(state[2]))

    return state, curr_left, curr_right


def run_single_command(left_power, right_power, state, prev_left, prev_right):
    """
    Run a single motor command for 3 seconds while continuously updating odometry.
    
    Args:
        left_power: left motor power (-1 to 1)
        right_power: right motor power (-1 to 1)
        state: numpy array [x, y, theta]
        prev_left: previous left encoder position
        prev_right: previous right encoder position
    
    Returns:
        state: updated numpy array [x, y, theta]
        prev_left: current left encoder position
        prev_right: current right encoder position
    """
    print(f"Running motors: L={left_power}, R={right_power}")

    left_motor.power_command = LEFT_DIR * left_power
    right_motor.power_command = RIGHT_DIR * right_power

    start = time.time()
    while time.time() - start < 3.0:
        state, prev_left, prev_right = update_odometry_rk4(
            state, prev_left, prev_right
        )
        time.sleep(DT)

    left_motor.power_command = 0
    right_motor.power_command = 0

    x, y, theta = state
    print(f"x={y:.2f} in, y={x:.2f} in, θ={np.degrees(theta):.1f}°\n")
    return state, prev_left, prev_right


# ================= MAIN =================
if __name__ == "__main__":
    print("Starting odometry with RK4 integration (numpy version)")
    input("Place robot at (0,0,0). Press Enter...\n")

    # Initialize state as numpy array [x, y, theta]
    state = np.array([0.0, 0.0, 0.0])
    prev_left = left_motor.position
    prev_right = right_motor.position

    try:
        state, prev_left, prev_right = run_single_command(
            LEFT_POWER, RIGHT_POWER,
            state,
            prev_left, prev_right
        )

        x, y, theta = state
        print("=" * 50)
        print(f"FINAL POSITION: x={y:.2f} in, y={-x:.2f} in")
        print(f"({x}, {-y})")
        print("=" * 50)

    except KeyboardInterrupt:
        left_motor.power_command = 0
        right_motor.power_command = 0
        print("Stopped.")