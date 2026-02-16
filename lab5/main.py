"""
File:           main.py
Description:    Lab 5 main script — ties together map, planner, and follower.
                Run this at demo time.

Usage:
    python main.py
    python main.py --dry-run    (test without hardware)
"""

import sys
import math
import numpy as np
from map_grid import build_occupancy_grid, visualize_grid, EASY_OBSTACLES
from path_planner import plan_path
from path_follower import (
    execute_path, heading_from_cardinal, compute_commands, print_commands
)


# ============================================================
# CONFIGURATION
# ============================================================
ROBOT_SIZE = 6            # robot width/length in inches — MEASURE YOUR ROBOT
# C-space inflation radius — must be < ~3-4" so demo start/goal aren't blocked
ROBOT_RADIUS = ROBOT_SIZE / 2  # 3" for a 6" robot
RESOLUTION = 4            # grid cells per inch (increase for accuracy)
OBSTACLES = EASY_OBSTACLES


def get_user_input():
    """Get start, goal, and heading from user at demo time."""
    print("=" * 50)
    print("  Lab 5 - Path Planning Demo")
    print("=" * 50)

    sx = float(input("Start X (inches): "))
    sy = float(input("Start Y (inches): "))
    gx = float(input("Goal X (inches):  "))
    gy = float(input("Goal Y (inches):  "))
    direction = input("Initial heading (N/E/S/W): ").strip().upper()

    return (sx, sy), (gx, gy), direction


def init_hardware():
    """Initialize MotorGo Plink and return (left_motor, right_motor)."""
    from motorgo.plink import Plink, ControlMode

    plink = Plink()
    plink.connect()

    left_motor = plink.channel4
    right_motor = plink.channel1

    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    left_motor.power_command = 0
    right_motor.power_command = 0

    print("Hardware initialized (Plink connected)")
    return left_motor, right_motor


def main():
    dry_run = '--dry-run' in sys.argv

    start, goal, direction = get_user_input()
    heading = heading_from_cardinal(direction)

    print(f"\nStart: {start}, Goal: {goal}, Heading: {direction}")
    print(f"Robot radius (c-space): {ROBOT_RADIUS:.1f} inches")
    print(f"Grid resolution: {RESOLUTION} cells/inch")
    print("\nPlanning path...")

    waypoints, grid = plan_path(start, goal, RESOLUTION, ROBOT_RADIUS, OBSTACLES)

    if waypoints is None:
        print("FAILED: No path found! Check start/goal positions.")
        visualize_grid(grid, RESOLUTION, start=start, goal=goal)
        return

    print(f"Path found with {len(waypoints)} waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  {i}: ({wp[0]:.1f}, {wp[1]:.1f})")

    # Preview the movement plan
    commands = compute_commands(waypoints, heading)
    print_commands(commands)

    # Show the path to the TA (required before robot moves)
    print("\n>>> Displaying path — show this to the TA <<<")
    visualize_grid(grid, RESOLUTION, path=waypoints, start=start, goal=goal)

    # After TA approves, execute
    if dry_run:
        print("\n[DRY RUN MODE] Simulating execution...")
        execute_path(waypoints, heading, dry_run=True)
    else:
        proceed = input("\nTA approved? Execute path? (y/n): ").strip().lower()
        if proceed == 'y':
            left_motor, right_motor = init_hardware()
            execute_path(waypoints, heading, left_motor, right_motor)
        else:
            print("Path execution cancelled.")


if __name__ == '__main__':
    main()
