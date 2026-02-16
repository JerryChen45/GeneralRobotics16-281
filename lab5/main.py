"""
Usage:
    python main.py                        (on robot: plan + execute)
    python main.py --visualize --dry-run  (on laptop: plan + show viz to TA)
    python main.py --dry-run              (test without hardware)
"""

import sys
import math
import numpy as np

# Try importing matplotlib (only available on laptop, not robot)
try:
    from map_grid import visualize_grid
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

from map_grid import (
    Easy_Obstacles, Hard_Obstacles,
    Robot_Radius, Resolution
)
from path_planner import plan_path
from path_follower import (
    execute_path, heading_from_cardinal, compute_commands, print_commands
)
# Change to Hard_Obstacles for hard course
Obstacles = Easy_Obstacles


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
    visualize = '--visualize' in sys.argv

    start, goal, direction = get_user_input()
    heading = heading_from_cardinal(direction)

    print(f"\nStart: {start}, Goal: {goal}, Heading: {direction}")
    print(f"Robot radius (c-space): {Robot_Radius:.1f} inches")
    print(f"Grid resolution: {Resolution} cells/inch")
    print("\nPlanning path...")

    waypoints, grids = plan_path(start, goal, Resolution, Robot_Radius, Obstacles)
    inflated_grid, original_grid = grids

    if waypoints is None:
        print("FAILED: No path found! Check start/goal positions.")
        if visualize and HAS_MATPLOTLIB:
            visualize_grid(inflated_grid, original_grid, start=start, goal=goal)
        return

    print(f"Path found with {len(waypoints)} waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  {i}: ({wp[0]:.1f}, {wp[1]:.1f})")

    # Preview the movement plan
    commands = compute_commands(waypoints, heading)
    print_commands(commands)

    # Show the path to the TA (required before robot moves)
    if visualize and HAS_MATPLOTLIB:
        print("\n>>> Displaying path — show this to the TA <<<")
        visualize_grid(inflated_grid, original_grid, path=waypoints, start=start, goal=goal)
    elif visualize and not HAS_MATPLOTLIB:
        print("\n[WARNING] Matplotlib not available — cannot visualize path")
    else:
        print("\n[Path computed — use --visualize on laptop to show TA]")

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
