"""
File:           path_follower.py
Description:    Proportional steering path follower for Lab 5.
                Steers toward each waypoint while driving — no point turns.
                Uses MotorGo Plink with RK4 odometry from Lab 3.
"""

import math
import time
import numpy as np

# ============================================================
# CONFIGURATION — from your Lab 3 code
# ============================================================
WHEEL_RADIUS = 1.125
       # inches
WHEELBASE = 6.5           # inches (distance between wheel centers)
TICKS_PER_REV = 6.4845
DT = 0.01                 # odometry update interval (seconds)

# Motor direction conventions (from Lab 3)
LEFT_MOTOR_DIR = -1       # left motor physically flipped
RIGHT_MOTOR_DIR = 1

# Encoder sign conventions (from Lab 3)
LEFT_ENC_SIGN = 1
RIGHT_ENC_SIGN = 1

# ============================================================
# STEERING TUNING PARAMETERS
# ============================================================
BASE_POWER = 0.5          # forward power (0 to 1) — tune for speed vs accuracy
KP_STEERING = 0.01         # proportional gain for heading correction — tune this
WAYPOINT_TOLERANCE = 2.0  # inches — how close before advancing to next waypoint
GOAL_TOLERANCE = 1.5      # inches — how close to final goal before stopping


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_from_cardinal(direction):
    """
    Convert a cardinal direction string to radians.
    'N' = +y = pi/2, 'E' = +x = 0, 'S' = -y = -pi/2, 'W' = -x = pi
    """
    mapping = {
        'N': math.pi / 2,
        'E': 0.0,
        'S': -math.pi / 2,
        'W': math.pi,
    }
    return mapping.get(direction.upper(), 0.0)


def rk4_step(state, v, omega, dt):
    """RK4 integration step for odometry (from Lab 3)."""
    def f(s):
        return np.array([
            v * math.cos(s[2]),
            v * math.sin(s[2]),
            omega
        ])

    k1 = f(state)
    k2 = f(state + 0.5 * dt * k1)
    k3 = f(state + 0.5 * dt * k2)
    k4 = f(state + dt * k3)

    state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    state[2] = normalize_angle(state[2])
    return state


def update_odometry(state, prev_left, prev_right, left_motor, right_motor):
    """Update robot pose using encoder readings (from Lab 3)."""
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

    state = rk4_step(state, v, omega, DT)
    return state, curr_left, curr_right


def distance_to(state, target):
    """Euclidean distance from current position to target."""
    return math.sqrt((target[0] - state[0])**2 + (target[1] - state[1])**2)


def heading_to(state, target):
    """Heading angle from current position to target."""
    return math.atan2(target[1] - state[1], target[0] - state[0])


def execute_path(waypoints, start_heading, left_motor=None, right_motor=None,
                 dry_run=False):
    """
    Follow waypoints using proportional steering.
    Adjusts wheel speeds to curve toward each waypoint — no stopping between segments.

    Args:
        waypoints: list of (x, y) in inches
        start_heading: initial heading in radians
        left_motor: Plink motor channel (channel1)
        right_motor: Plink motor channel (channel3)
        dry_run: if True, simulate without hardware (print commands only)
    """
    if len(waypoints) < 2:
        print("Need at least 2 waypoints!")
        return

    # Initialize state at first waypoint with given heading
    state = np.array([waypoints[0][0], waypoints[0][1], start_heading])

    print(f"\nStarting path execution:")
    print(f"  Position: ({state[0]:.1f}, {state[1]:.1f})")
    print(f"  Heading:  {math.degrees(state[2]):.1f} deg")
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Base power: {BASE_POWER}, Kp: {KP_STEERING}")

    if dry_run:
        # Just print the plan
        for i in range(len(waypoints) - 1):
            dx = waypoints[i+1][0] - waypoints[i][0]
            dy = waypoints[i+1][1] - waypoints[i][1]
            dist = math.sqrt(dx**2 + dy**2)
            heading = math.degrees(math.atan2(dy, dx))
            print(f"  Segment {i+1}: -> ({waypoints[i+1][0]:.1f}, {waypoints[i+1][1]:.1f}) "
                  f"dist={dist:.1f}\" heading={heading:.1f} deg")
        print("\n[DRY RUN] No motors activated.")
        return

    # Initialize encoder tracking
    prev_left = left_motor.position
    prev_right = right_motor.position

    wp_index = 1  # current target waypoint (skip start)
    start_time = time.time()

    try:
        while wp_index < len(waypoints):
            target = waypoints[wp_index]
            is_final = (wp_index == len(waypoints) - 1)
            tolerance = GOAL_TOLERANCE if is_final else WAYPOINT_TOLERANCE

            dist = distance_to(state, target)

            # Check if we've reached this waypoint
            if dist < tolerance:
                print(f"  Reached waypoint {wp_index}: ({target[0]:.1f}, {target[1]:.1f}) "
                      f"[error: {dist:.2f}\"]")
                wp_index += 1
                continue

            # Compute heading error
            desired_heading = heading_to(state, target)
            heading_error = normalize_angle(desired_heading - state[2])

            # Proportional steering: adjust wheel powers
            steering = KP_STEERING * heading_error
            left_power = BASE_POWER - steering
            right_power = BASE_POWER + steering

            # Clamp powers to [-1, 1]
            left_power = max(-1.0, min(1.0, left_power))
            right_power = max(-1.0, min(1.0, right_power))

            # Send to motors (applying direction conventions)
            left_motor.power_command = LEFT_MOTOR_DIR * left_power
            right_motor.power_command = RIGHT_MOTOR_DIR * right_power

            # Update odometry
            time.sleep(DT)
            state, prev_left, prev_right = update_odometry(
                state, prev_left, prev_right, left_motor, right_motor
            )

        # Stop motors
        left_motor.power_command = 0
        right_motor.power_command = 0

        elapsed = time.time() - start_time
        final_dist = distance_to(state, waypoints[-1])
        print(f"\nPath complete!")
        print(f"  Final position: ({state[0]:.2f}, {state[1]:.2f})")
        print(f"  Final heading:  {math.degrees(state[2]):.1f} deg")
        print(f"  L2 error:       {final_dist:.2f} inches")
        print(f"  Time:           {elapsed:.1f} seconds")

    except KeyboardInterrupt:
        left_motor.power_command = 0
        right_motor.power_command = 0
        print("\nStopped by user!")
        print(f"  Position at stop: ({state[0]:.2f}, {state[1]:.2f})")


def compute_commands(waypoints, start_heading):
    """
    Preview the path as a list of (turn_angle, distance) for display.
    Used for showing the TA the planned path before execution.
    """
    commands = []
    current_heading = start_heading

    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        dx, dy = x2 - x1, y2 - y1
        distance = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)
        turn_angle = normalize_angle(target_heading - current_heading)
        commands.append((turn_angle, distance))
        current_heading = target_heading

    return commands


def print_commands(commands):
    """Print the command sequence in a human-readable format."""
    print(f"\n{'Step':<6} {'Turn (deg)':<14} {'Drive (in)':<12}")
    print("-" * 32)
    for i, (turn, dist) in enumerate(commands):
        print(f"{i+1:<6} {math.degrees(turn):>+10.1f}    {dist:>8.1f}")
    total_dist = sum(d for _, d in commands)
    print(f"\nTotal distance: {total_dist:.1f} inches")
    print(f"Number of segments: {len(commands)}")


if __name__ == '__main__':
    # Dry-run test with example waypoints
    test_waypoints = [(40, 32), (30, 20), (20, 10), (16, 8)]
    start_dir = 'N'

    heading = heading_from_cardinal(start_dir)
    print(f"Start heading: {start_dir} ({math.degrees(heading):.0f} deg)")

    commands = compute_commands(test_waypoints, heading)
    print_commands(commands)

    execute_path(test_waypoints, heading, dry_run=True)
