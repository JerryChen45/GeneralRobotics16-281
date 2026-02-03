import time
import math
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
TICKS_PER_REV = 5   # <-- your calibrated value
DT = 0.01              # integration time step (seconds)
# ====================================

# ========== MOTOR DIRECTION ==========
LEFT_DIR = -1    # left motor is flipped
RIGHT_DIR = 1    # right motor is normal
# ====================================

# ========== COMMAND ==========
LEFT_POWER = 1
RIGHT_POWER = 1
# =============================


def update_odometry_rk4(x, y, theta, prev_left, prev_right):
    """
    Update odometry using 4th-order Runge-Kutta (RK4) integration.
    More accurate than basic Euler integration, especially for curved paths.
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

    # RK4 Integration
    # k1: derivatives at current state
    k1_x = v * math.cos(theta)
    k1_y = v * math.sin(theta)
    k1_theta = omega
    
    # k2: derivatives at midpoint using k1
    k2_x = v * math.cos(theta + 0.5 * DT * k1_theta)
    k2_y = v * math.sin(theta + 0.5 * DT * k1_theta)
    k2_theta = omega
    
    # k3: derivatives at midpoint using k2
    k3_x = v * math.cos(theta + 0.5 * DT * k2_theta)
    k3_y = v * math.sin(theta + 0.5 * DT * k2_theta)
    k3_theta = omega
    
    # k4: derivatives at endpoint using k3
    k4_x = v * math.cos(theta + DT * k3_theta)
    k4_y = v * math.sin(theta + DT * k3_theta)
    k4_theta = omega
    
    # Weighted average of slopes
    x += (DT / 6.0) * (k1_x + 2*k2_x + 2*k3_x + k4_x)
    y += (DT / 6.0) * (k1_y + 2*k2_y + 2*k3_y + k4_y)
    theta += (DT / 6.0) * (k1_theta + 2*k2_theta + 2*k3_theta + k4_theta)
    
    # Normalize theta to [-pi, pi]
    theta = math.atan2(math.sin(theta), math.cos(theta))

    return x, y, theta, curr_left, curr_right


def run_single_command(left_power, right_power, x, y, theta, prev_left, prev_right):
    """
    Run a single motor command for 3 seconds while continuously updating odometry.
    """
    print(f"Running motors: L={left_power}, R={right_power}")

    left_motor.power_command = LEFT_DIR * left_power
    right_motor.power_command = RIGHT_DIR * right_power

    start = time.time()
    while time.time() - start < 3.0:
        x, y, theta, prev_left, prev_right = update_odometry_rk4(
            x, y, theta, prev_left, prev_right
        )
        time.sleep(DT)

    left_motor.power_command = 0
    right_motor.power_command = 0

    print(f"x={x:.2f} in, y={y:.2f} in, θ={math.degrees(theta):.1f}°\n")
    return x, y, theta, prev_left, prev_right


# ================= MAIN =================
if __name__ == "__main__":
    print("Starting odometry with RK4 integration")
    input("Place robot at (0,0,0). Press Enter...\n")

    x = y = theta = 0.0
    prev_left = left_motor.position
    prev_right = right_motor.position

    try:
        x, y, theta, prev_left, prev_right = run_single_command(
            LEFT_POWER, RIGHT_POWER,
            x, y, theta,
            prev_left, prev_right
        )

        print("=" * 50)
        print(f"FINAL POSITION: x={x:.2f} in, y={y:.2f} in")
        print("=" * 50)

    except KeyboardInterrupt:
        left_motor.power_command = 0
        right_motor.power_command = 0
        print("Stopped.")