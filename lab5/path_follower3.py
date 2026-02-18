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
WHEEL_RADIUS = 1.125       # inches
WHEELBASE = 6.25           # inches (distance between wheel centers)
TICKS_PER_REV = 6.4845
DT = 0.01                 # odometry update interval (seconds)

# Motor direction conventions (from Lab 3)
LEFT_MOTOR_DIR = 1       # left motor physically flipped
RIGHT_MOTOR_DIR = -1

# Encoder sign conventions (from Lab 3)
LEFT_ENC_SIGN = -1
RIGHT_ENC_SIGN = 1

# ============================================================
# STEERING TUNING PARAMETERS
# ============================================================
BASE_POWER = 0.5          # forward power (0 to 1) — tune for speed vs accuracy
KP_STEERING = 7      # proportional gain for heading correction — tune this
WAYPOINT_TOLERANCE = 1  # inches — how close before advancing to next waypoint
GOAL_TOLERANCE = 1     # inches — how close to final goal before stopping

KD_STEERING = 0.006


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

def f(s, v, omega):
    return np.array([
        v * math.cos(s[2]),
        v * math.sin(s[2]),
        omega
    ])

def rk4_step(state, v, omega, dt):
    """RK4 integration step for odometry (from Lab 3)."""

    k1 = f(state, v, omega)
    k2 = f(state + 0.5 * dt * k1, v, omega)
    k3 = f(state + 0.5 * dt * k2, v, omega)
    k4 = f(state + dt * k3, v, omega)

    state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    state[2] = normalize_angle(state[2])
    return state


def update_odometry(state, prev_left, prev_right, left_motor, right_motor):
    curr_left = left_motor.position
    curr_right = right_motor.position

    delta_left = LEFT_ENC_SIGN * (curr_left - prev_left)
    delta_right = RIGHT_ENC_SIGN * (curr_right - prev_right)
    print(f"Encoders: L={curr_left:.2f} (Δ={delta_left:.2f}), R={curr_right:.2f} (Δ={delta_right:.2f})")

    omega_left_clicks = delta_left / DT
    omega_right_clicks = delta_right / DT
    omega_left_wheel = (omega_left_clicks / TICKS_PER_REV) * (2 * math.pi)
    omega_right_wheel = (omega_right_clicks / TICKS_PER_REV) * (2 * math.pi)
    
    v_left = omega_left_wheel * WHEEL_RADIUS
    v_right = omega_right_wheel * WHEEL_RADIUS

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
    time.sleep(1)
    print(f"Initial encoders: L={prev_left:.2f}, R={prev_right:.2f}")
    prev_left = left_motor.position
    prev_right = right_motor.position
    prev_heading_error = 0.0

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
            print(f"  HDG={math.degrees(state[2]):.1f}° desired={math.degrees(desired_heading):.1f}° err={math.degrees(heading_error):.1f}°")

            # Proportional steering: adjust wheel powers
            steering = KP_STEERING * heading_error + KD_STEERING * (heading_error - prev_heading_error) / DT
            prev_heading_error = heading_error
            left_power = BASE_POWER - 1 * steering if steering > 0 else BASE_POWER - steering
            right_power = BASE_POWER + 1 * steering if steering < 0 else BASE_POWER + steering

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
    import sys
    
    # Check if dry-run mode
    dry_run = '--dry-run' in sys.argv
    
    # Test waypoints
    test_waypoints = [(10.0, 25.0) , (10.875, 30.25) , (11.75, 35.5) , (12.0, 35.75) , (19.75, 36.75) , (27.5, 37.75) , (29.625, 41.875) , (31.75, 46.0) , (31.75, 47.5) , (32.0, 47.75) , (39.375, 47.75) , (46.75, 47.75) , (54.125, 47.75) , (61.5, 47.75) , (61.75, 47.5) , (62.0, 45.0)]
    # test_waypoints = [(y, -x) for x, y in test_waypoints]
    start_dir = 'S'
    
    heading = heading_from_cardinal(start_dir)

    print(f"Start heading: {start_dir} ({math.degrees(heading):.0f} deg)")
    
    commands = compute_commands(test_waypoints, heading)
    print_commands(commands)
    
    if dry_run:
        # Dry run - no motors
        execute_path(test_waypoints, heading, dry_run=True)
    else:
        # Real execution - initialize hardware
        print("\n*** REAL EXECUTION MODE ***")
        proceed = input("Initialize motors and run? (y/n): ").strip().lower()
        
        if proceed == 'y':
            from motorgo.plink import Plink, ControlMode
            
            # Initialize hardware
            plink = Plink()
            plink.connect()
            
            left_motor = plink.channel1
            right_motor = plink.channel4
            
            left_motor.control_mode = ControlMode.POWER
            right_motor.control_mode = ControlMode.POWER
            
            left_motor.power_command = 0
            right_motor.power_command = 0
            
            print("Hardware initialized!")
            
            # Execute path
            execute_path(test_waypoints, heading, left_motor, right_motor, dry_run=False)
        else:
            print("Cancelled.")