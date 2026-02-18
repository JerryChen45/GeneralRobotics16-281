# Map_grid is the first step in our pipeline. it converts the
# given obstacles into a grid representation, where each cell
# is marked as either free or occupied.

#on robot: python3 main.py
#on Laptop: python3 main.py --visualize --dry-run
import numpy as np

# Matplotlib only needed for visualization (laptop only, not robot)
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Map dimensions (Inches))
Map_Width = 72
Map_Height = 54

#robot 
Robot_Radius = 5.9
Resolution = 4  # per inch

# Easy course obstacles as (x_min, y_min, x_max, y_max)
Easy_Obstacles = [
    (8, 12, 14, 18),     # 6x6
    (18, 12, 24, 30),    # 6x18
    (16, 44, 22, 50),    # 6x6
    (38, 36, 56, 42),    # 18x6
]

# Hard course obstacles as (x_min, y_min, x_max, y_max)
Hard_Obstacles = [
    (6, 10.5, 12, 16.5),         # 6x6
    (23, 19, 41, 25),            # 18x6
    (33.5, 37.5, 46.5, 50.5),    # 14.33x4 @ 45°
    (38.25, 7.25, 49.75, 18.75), # 12.21x4 @ 45°
]

# Builds occupancy grid w/ C-space inflation
# Returns: (inflated_grid, original_grid) - both are True/False grids
def build_occupancy_grid(obstacles, resolution, robot_radius):

    cols = int(Map_Width * resolution)
    rows = int(Map_Height * resolution)

    # original obstacles grid (no inflation)
    original_grid = np.zeros((rows, cols), dtype=bool)
    for x_min, y_min, x_max, y_max in obstacles:
        col_start = max(0, int(x_min * resolution))
        col_end = min(cols, int(x_max * resolution))
        row_start = max(0, int(y_min * resolution))
        row_end = min(rows, int(y_max * resolution))
        original_grid[row_start:row_end, col_start:col_end] = True

    # c-space inflated grid
    inflated_grid = np.zeros((rows, cols), dtype=bool)
    for x_min, y_min, x_max, y_max in obstacles:
        cx_min = x_min - robot_radius
        cy_min = y_min - robot_radius
        cx_max = x_max + robot_radius
        cy_max = y_max + robot_radius

        # Convert to grid indices (clamp to map bounds)
        col_start = max(0, int(cx_min * resolution))
        col_end = min(cols, int(cx_max * resolution))
        row_start = max(0, int(cy_min * resolution))
        row_end = min(rows, int(cy_max * resolution))

        inflated_grid[row_start:row_end, col_start:col_end] = True

    # Wall boundaries - inflate for c-space
    wall_cells = max(1, int(robot_radius * resolution))
    inflated_grid[:wall_cells, :] = True          # bottom wall
    inflated_grid[-wall_cells:, :] = True         # top wall
    inflated_grid[:, :wall_cells] = True          # left wall
    inflated_grid[:, -wall_cells:] = True         # right wall

    return inflated_grid, original_grid


def grid_to_world(col, row, resolution):
    return col / resolution, row / resolution

def world_to_grid(x, y, resolution):
    return int(x * resolution), int(y * resolution)

def visualize_grid(inflated_grid, original_grid, path=None, start=None, goal=None):
    """
    Visualize grid on LAPTOP ONLY (for TA demo).
    Robot loads pre-computed waypoints instead of running this.
    """
    if not HAS_MATPLOTLIB:
        print("ERROR: Matplotlib not installed. Cannot visualize grid.")
        return

    fig, ax = plt.subplots(1, 1, figsize=(10, 7.5))

    display = np.zeros((*inflated_grid.shape, 3), dtype=np.uint8)
    display[~inflated_grid] = [240, 240, 240]    # free space in light gray
    display[inflated_grid] = [255, 180, 100]     # c-space inflated areas in orange
    display[original_grid] = [180, 50, 50]       # original obstacles in dark red

    ax.imshow(display, origin='lower', extent=[0, Map_Width, 0, Map_Height])

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
    ax.set_title('Lab 5 - Occupancy Grid (Dark Red = Obstacles, Orange = C-Space Inflation)')
    ax.legend()
    ax.set_xlim(0, Map_Width)
    ax.set_ylim(0, Map_Height)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # They will be at least 5 inch from all obstacles
    # (including the walls). These coordinates will be with respect to a bottom-left origin.

    # Build grids: inflated (for path planning) and original (for visualization)
    inflated_grid, original_grid = build_occupancy_grid(Hard_Obstacles, Resolution, Robot_Radius)

    print(f"Grid shape: {inflated_grid.shape} (rows x cols)")
    print(f"Blocked cells: {inflated_grid.sum()} / {inflated_grid.size} ({100*inflated_grid.sum()/inflated_grid.size:.1f}%)")

    # Visualize (laptop only - for TA demo)
    visualize_grid(inflated_grid, original_grid)
