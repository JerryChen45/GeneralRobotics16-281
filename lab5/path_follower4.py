"""
File:           path_follower.py
Description:    Pure Pursuit path follower for Lab 5.
                Uses a lookahead circle to smoothly follow waypoints.
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
DT = 0.01                  # odometry update interval (seconds)

# Motor direction conventions (from Lab 3)
LEFT_MOTOR_DIR = 1         # left motor physically flipped
RIGHT_MOTOR_DIR = -1

# Encoder sign conventions (from Lab 3)
LEFT_ENC_SIGN = -1
RIGHT_ENC_SIGN = 1

# ============================================================
# PURE PURSUIT TUNING PARAMETERS
# ============================================================
BASE_POWER = 0.35          # forward power (0 to 1) — tune for speed vs accuracy
KP_STEERING = 5.0          # proportional gain on heading error
LOOKAHEAD_DIST = 8.0       # inches — larger = smoother but cuts corners more
GOAL_TOLERANCE = 2.0       # inches — how close to final goal before stopping
WAYPOINT_TOLERANCE = 2.0   # inches — used to advance wp_index tracker


# ============================================================
# MATH / ODOMETRY HELPERS
# ============================================================

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
    """Euclidean distance from current position to target (x, y)."""
    return math.sqrt((target[0] - state[0])**2 + (target[1] - state[1])**2)


def heading_to(state, target):
    """Heading angle from current position to target (x, y)."""
    return math.atan2(target[1] - state[1], target[0] - state[0])


# ============================================================
# PURE PURSUIT LOOKAHEAD
# ============================================================

def find_lookahead_point(state, waypoints, wp_index, lookahead):
    """
    Find the point on the path exactly `lookahead` inches ahead of the robot.
    Searches forward from wp_index - 1 along the remaining path segments.
    Falls back to the final waypoint if no intersection is found.

    Args:
        state:      current robot state [x, y, theta]
        waypoints:  list of (x, y) world coordinates
        wp_index:   current target waypoint index
        lookahead:  lookahead distance in inches

    Returns:
        (x, y) lookahead point
    """
    x, y = state[0], state[1]

    # Search from the segment leading into wp_index onward
    for i in range(max(0, wp_index - 1), len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        dx, dy = x2 - x1, y2 - y1

        # Vector from segment start to robot
        fx = x1 - x
        fy = y1 - y

        a = dx*dx + dy*dy
        b = 2.0 * (fx*dx + fy*dy)
        c = fx*fx + fy*fy - lookahead**2
        disc = b*b - 4.0*a*c

        if disc < 0 or a < 1e-10:
            continue  # no intersection with this segment

        disc_sqrt = math.sqrt(disc)
        # Prefer t2 (further along segment = further ahead on path)
        for t in [(-b + disc_sqrt) / (2*a), (-b - disc_sqrt) / (2*a)]:
            if 0.0 <= t <= 1.0:
                return (x1 + t * dx, y1 + t * dy)

    # Fallback: aim at the final waypoint
    return waypoints[-1]


# ============================================================
# MAIN EXECUTION
# ============================================================

def execute_path(waypoints, start_heading, left_motor=None, right_motor=None,
                 dry_run=False):
    """
    Follow waypoints using Pure Pursuit steering.
    Continuously steers toward a lookahead point on the path for smooth curves.

    Args:
        waypoints:      list of (x, y) in inches
        start_heading:  initial heading in radians
        left_motor:     Plink motor channel (channel1)
        right_motor:    Plink motor channel (channel4)
        dry_run:        if True, simulate without hardware
    """
    if len(waypoints) < 2:
        print("Need at least 2 waypoints!")
        return

    state = np.array([waypoints[0][0], waypoints[0][1], start_heading])

    print(f"\nStarting Pure Pursuit path execution:")
    print(f"  Position:     ({state[0]:.1f}, {state[1]:.1f})")
    print(f"  Heading:      {math.degrees(state[2]):.1f} deg")
    print(f"  Waypoints:    {len(waypoints)}")
    print(f"  Base power:   {BASE_POWER}")
    print(f"  Kp steering:  {KP_STEERING}")
    print(f"  Lookahead:    {LOOKAHEAD_DIST}\"")

    if dry_run:
        for i in range(len(waypoints) - 1):
            dx = waypoints[i+1][0] - waypoints[i][0]
            dy = waypoints[i+1][1] - waypoints[i][1]
            dist = math.sqrt(dx**2 + dy**2)
            hdg = math.degrees(math.atan2(dy, dx))
            print(f"  Segment {i+1}: -> ({waypoints[i+1][0]:.1f}, {waypoints[i+1][1]:.1f}) "
                  f"dist={dist:.1f}\" heading={hdg:.1f} deg")
        print("\n[DRY RUN] No motors activated.")
        return

    # Initialize encoders
    prev_left = left_motor.position
    prev_right = right_motor.position
    time.sleep(1)
    print(f"Initial encoders: L={prev_left:.2f}, R={prev_right:.2f}")
    prev_left = left_motor.position
    prev_right = right_motor.position

    wp_index = 1      # tracks which waypoint we're approaching (for lookahead search)
    start_time = time.time()

    try:
        while True:
            # ── Termination: close enough to final goal ──────────────────
            final_dist = distance_to(state, waypoints[-1])
            if final_dist < GOAL_TOLERANCE:
                print(f"  Reached goal! [error: {final_dist:.2f}\"]")
                break

            # ── Advance wp_index so lookahead searches forward ───────────
            while wp_index < len(waypoints) - 1:
                if distance_to(state, waypoints[wp_index]) < WAYPOINT_TOLERANCE:
                    print(f"  Passed waypoint {wp_index}: "
                          f"({waypoints[wp_index][0]:.1f}, {waypoints[wp_index][1]:.1f})")
                    wp_index += 1
                else:
                    break

            # ── Pure Pursuit: find lookahead point ───────────────────────
            lookahead_pt = find_lookahead_point(state, waypoints, wp_index, LOOKAHEAD_DIST)

            desired_heading = heading_to(state, lookahead_pt)
            heading_error = normalize_angle(desired_heading - state[2])

            print(f"  HDG={math.degrees(state[2]):.1f}°  "
                  f"desired={math.degrees(desired_heading):.1f}°  "
                  f"err={math.degrees(heading_error):.1f}°  "
                  f"lookahead=({lookahead_pt[0]:.1f}, {lookahead_pt[1]:.1f})  "
                  f"dist_to_goal={final_dist:.1f}\"")

            # ── Compute wheel powers ─────────────────────────────────────
            steering = KP_STEERING * heading_error
            left_power = BASE_POWER - steering
            right_power = BASE_POWER + steering

            # Normalize so max is capped at 1.0 while preserving the ratio
            max_power = max(abs(left_power), abs(right_power))
            if max_power > 1.0:
                left_power /= max_power
                right_power /= max_power

            # Apply motor direction conventions
            left_motor.power_command = LEFT_MOTOR_DIR * left_power
            right_motor.power_command = RIGHT_MOTOR_DIR * right_power

            # ── Odometry update ──────────────────────────────────────────
            time.sleep(DT)
            state, prev_left, prev_right = update_odometry(
                state, prev_left, prev_right, left_motor, right_motor
            )

        # Stop
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


# ============================================================
# COMMAND PREVIEW UTILITIES
# ============================================================

def compute_commands(waypoints, start_heading):
    """Preview the path as a list of (turn_angle, distance) for display."""
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


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == '__main__':
    import sys

    dry_run = '--dry-run' in sys.argv

    test_waypoints = [
        (7.0, 47.0), (10.125, 44.0), (13.25, 41.0), (20.625, 41.0),
        (28.0, 41.0), (33.875, 44.0), (39.75, 47.0), (46.5, 47.0),
        (53.25, 47.0), (60.0, 47.0)
    ]
    start_dir = 'N'

    heading = heading_from_cardinal(start_dir)
    print(f"Start heading: {start_dir} ({math.degrees(heading):.0f} deg)")

    commands = compute_commands(test_waypoints, heading)
    print_commands(commands)

    if dry_run:
        execute_path(test_waypoints, heading, dry_run=True)
    else:
        print("\n*** REAL EXECUTION MODE ***")
        proceed = input("Initialize motors and run? (y/n): ").strip().lower()

        if proceed == 'y':
            from motorgo.plink import Plink, ControlMode

            plink = Plink()
            plink.connect()

            left_motor = plink.channel1
            right_motor = plink.channel4

            left_motor.control_mode = ControlMode.POWER
            right_motor.control_mode = ControlMode.POWER
            left_motor.power_command = 0
            right_motor.power_command = 0

            print("Hardware initialized!")
            execute_path(test_waypoints, heading, left_motor, right_motor, dry_run=False)
        else:
            print("Cancelled.")