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
TICKS_PER_REV = 6.32   # <-- your calibrated value
# ====================================

# ========== MOTOR DIRECTION ==========
LEFT_DIR = -1    # left motor is flipped
RIGHT_DIR = 1    # right motor is normal
# ====================================

# ========== COMMAND ==========
LEFT_POWER = 1
RIGHT_POWER = 1
# =============================


def update_odometry(x, y, theta, prev_left, prev_right):
    curr_left = left_motor.position
    curr_right = right_motor.position

    delta_left = curr_left - prev_left
    delta_right = curr_right - prev_right

    left_dist = (delta_left / TICKS_PER_REV) * (2 * math.pi * WHEEL_RADIUS)
    right_dist = (delta_right / TICKS_PER_REV) * (2 * math.pi * WHEEL_RADIUS)

    center_dist = (left_dist + right_dist) / 2.0
    delta_theta = (right_dist - left_dist) / WHEELBASE

    if abs(delta_theta) < 1e-6:
        dx = center_dist * math.cos(theta)
        dy = center_dist * math.sin(theta)
    else:
        R = center_dist / delta_theta
        dx = R * (math.sin(theta + delta_theta) - math.sin(theta))
        dy = -R * (math.cos(theta + delta_theta) - math.cos(theta))

    x += dx
    y += dy
    theta += delta_theta
    theta = math.atan2(math.sin(theta), math.cos(theta))

    return x, y, theta, curr_left, curr_right


def run_single_command(left_power, right_power, x, y, theta, prev_left, prev_right):
    print(f"Running motors: L={left_power}, R={right_power}")

    left_motor.power_command = LEFT_DIR * left_power
    right_motor.power_command = RIGHT_DIR * right_power

    start = time.time()
    while time.time() - start < 3.0:
        x, y, theta, prev_left, prev_right = update_odometry(
            x, y, theta, prev_left, prev_right
        )
        time.sleep(0.01)

    left_motor.power_command = 0
    right_motor.power_command = 0

    print(f"x={x:.2f} in, y={y:.2f} in, θ={math.degrees(theta):.1f}°\n")
    return x, y, theta, prev_left, prev_right


# ================= MAIN =================
if __name__ == "__main__":
    print("Starting odometry")
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
