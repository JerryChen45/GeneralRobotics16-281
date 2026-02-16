"""
File:           map_grid.py
Description:    Generates an occupancy grid with c-space inflation for Lab 5
"""

import numpy as np
import matplotlib.pyplot as plt

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# Map dimensions in inches
MAP_WIDTH = 72
MAP_HEIGHT = 54

# Easy course obstacles as (x_min, y_min, x_max, y_max)
EASY_OBSTACLES = [
    (8, 12, 14, 18),     # 6x6
    (18, 12, 24, 30),    # 6x18
    (16, 44, 22, 50),    # 6x6
    (38, 36, 56, 42),    # 18x6
]

# Wall boundaries (the border itself is an obstacle for c-space)
# Walls are at x=0, x=72, y=0, y=54


def build_occupancy_grid(obstacles, resolution, robot_radius):
    """
    Build a 2D boolean occupancy grid with c-space inflation.

    Args:
        obstacles: list of (x_min, y_min, x_max, y_max) tuples
        resolution: grid cells per inch
        robot_radius: inflation radius in inches (half the robot diagonal)

    Returns:
        grid: 2D numpy bool array (True = blocked), shape (rows, cols)
               grid[row][col] corresponds to position (col/resolution, row/resolution)
    """
    cols = int(MAP_WIDTH * resolution)
    rows = int(MAP_HEIGHT * resolution)
    grid = np.zeros((rows, cols), dtype=bool)

    for x_min, y_min, x_max, y_max in obstacles:
        # Inflate by robot_radius for c-space
        cx_min = x_min - robot_radius
        cy_min = y_min - robot_radius
        cx_max = x_max + robot_radius
        cy_max = y_max + robot_radius

        # Convert to grid indices (clamp to map bounds)
        col_start = max(0, int(cx_min * resolution))
        col_end = min(cols, int(cx_max * resolution))
        row_start = max(0, int(cy_min * resolution))
        row_end = min(rows, int(cy_max * resolution))

        grid[row_start:row_end, col_start:col_end] = True

    # Inflate walls (robot can't get within robot_radius of any wall)
    wall_cells = max(1, int(robot_radius * resolution))
    grid[:wall_cells, :] = True          # bottom wall
    grid[-wall_cells:, :] = True         # top wall
    grid[:, :wall_cells] = True          # left wall
    grid[:, -wall_cells:] = True         # right wall

    return grid


def grid_to_world(col, row, resolution):
    """Convert grid indices to world coordinates (inches)."""
    return col / resolution, row / resolution


def world_to_grid(x, y, resolution):
    """Convert world coordinates (inches) to grid indices."""
    return int(x * resolution), int(y * resolution)


def visualize_grid(grid, resolution, path=None, start=None, goal=None):
    """
    Display the occupancy grid with optional path overlay.

    Args:
        grid: 2D bool array from build_occupancy_grid
        resolution: cells per inch
        path: optional list of (x, y) world coordinates
        start: optional (x, y) start position
        goal: optional (x, y) goal position
    """
    if not HAS_MATPLOTLIB:
        print("[SKIP] matplotlib not available — skipping visualization.")
        if path:
            print("Planned waypoints:")
            for i, p in enumerate(path):
                print(f"  {i}: ({p[0]:.1f}, {p[1]:.1f})")
        return

    fig, ax = plt.subplots(1, 1, figsize=(10, 7.5))

    # Show grid (origin='lower' so y=0 is at bottom)
    display = np.zeros((*grid.shape, 3), dtype=np.uint8)
    display[grid] = [200, 50, 50]       # obstacles in red
    display[~grid] = [240, 240, 240]    # free space in light gray

    ax.imshow(display, origin='lower', extent=[0, MAP_WIDTH, 0, MAP_HEIGHT])

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, 'b-', linewidth=2, label='Path')

    if start:
        ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
    if goal:
        ax.plot(goal[0], goal[1], 'rx', markersize=10, markeredgewidth=3, label='Goal')

    ax.set_xlabel('X (inches)')
    ax.set_ylabel('Y (inches)')
    ax.set_title('Lab 5 - Occupancy Grid with C-Space')
    ax.legend()
    ax.set_xlim(0, MAP_WIDTH)
    ax.set_ylim(0, MAP_HEIGHT)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # Robot c-space radius — half side length of robot
    # Must be < ~3-4" so demo start/goal positions aren't inside inflated obstacles
    ROBOT_RADIUS = 8.867  # 3" for a 6" robot
    RESOLUTION = 4  # 4 cells per inch

    grid = build_occupancy_grid(EASY_OBSTACLES, RESOLUTION, ROBOT_RADIUS)
    print(f"Grid shape: {grid.shape} (rows x cols)")
    print(f"Blocked cells: {grid.sum()} / {grid.size} ({100*grid.sum()/grid.size:.1f}%)")

    visualize_grid(grid, RESOLUTION)
