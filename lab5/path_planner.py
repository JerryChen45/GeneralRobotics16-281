"""
File:           path_planner.py
Description:    A* path planner on an occupancy grid + path smoothing for Lab 5
"""

import heapq
import math
import numpy as np
from map_grid import (
    build_occupancy_grid, world_to_grid, grid_to_world,
    visualize_grid, EASY_OBSTACLES, MAP_WIDTH, MAP_HEIGHT
)


def astar(grid, start_grid, goal_grid):
    """
    A* search on a 2D boolean occupancy grid.

    Args:
        grid: 2D numpy bool array (True = blocked)
        start_grid: (col, row) start cell
        goal_grid: (col, row) goal cell

    Returns:
        path: list of (col, row) grid cells from start to goal, or None if no path
    """
    rows, cols = grid.shape
    sc, sr = start_grid
    gc, gr = goal_grid

    if grid[sr, sc] or grid[gr, gc]:
        print("ERROR: Start or goal is inside an obstacle!")
        return None

    # 8-connected neighbors (col_offset, row_offset, cost)
    neighbors = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
    ]

    def heuristic(c, r):
        return math.sqrt((c - gc)**2 + (r - gr)**2)

    # Priority queue: (f_score, col, row)
    open_set = [(heuristic(sc, sr), sc, sr)]
    g_score = {(sc, sr): 0.0}
    came_from = {}

    while open_set:
        f, c, r = heapq.heappop(open_set)

        if (c, r) == (gc, gr):
            # Reconstruct path
            path = [(gc, gr)]
            while (path[-1]) in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            return path

        current_g = g_score.get((c, r), float('inf'))
        if f > current_g + heuristic(c, r) + 1e-6:
            continue  # stale entry

        for dc, dr, cost in neighbors:
            nc, nr = c + dc, r + dr
            if 0 <= nc < cols and 0 <= nr < rows and not grid[nr, nc]:
                new_g = current_g + cost
                if new_g < g_score.get((nc, nr), float('inf')):
                    g_score[(nc, nr)] = new_g
                    came_from[(nc, nr)] = (c, r)
                    heapq.heappush(open_set, (new_g + heuristic(nc, nr), nc, nr))

    print("ERROR: No path found!")
    return None


def smooth_path(path, grid):
    """
    Line-of-sight path smoothing. Remove intermediate waypoints
    when a straight line between two points doesn't cross obstacles.

    Args:
        path: list of (col, row) grid cells
        grid: 2D numpy bool array

    Returns:
        smoothed: list of (col, row) with unnecessary waypoints removed
    """
    if not path or len(path) <= 2:
        return path

    smoothed = [path[0]]
    i = 0

    while i < len(path) - 1:
        # Find the farthest point we can reach in a straight line
        farthest = i + 1
        for j in range(len(path) - 1, i, -1):
            if line_of_sight(grid, path[i], path[j]):
                farthest = j
                break
        smoothed.append(path[farthest])
        i = farthest

    return smoothed


def line_of_sight(grid, p1, p2):
    """
    Check if a straight line from p1 to p2 is collision-free using Bresenham's.
    """
    c1, r1 = p1
    c2, r2 = p2
    dc = abs(c2 - c1)
    dr = abs(r2 - r1)
    sc = 1 if c1 < c2 else -1
    sr = 1 if r1 < r2 else -1
    err = dc - dr
    c, r = c1, r1

    while True:
        if grid[r, c]:
            return False
        if c == c2 and r == r2:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr

    return True


def plan_path(start_world, goal_world, resolution, robot_radius, obstacles=None):
    """
    Full pipeline: build grid, run A*, smooth, return world-coordinate waypoints.

    Args:
        start_world: (x, y) in inches
        goal_world: (x, y) in inches
        resolution: grid cells per inch
        robot_radius: c-space inflation in inches
        obstacles: list of (x_min, y_min, x_max, y_max); defaults to EASY_OBSTACLES

    Returns:
        waypoints: list of (x, y) in inches, or None if no path
        grid: the occupancy grid (for visualization)
    """
    if obstacles is None:
        obstacles = EASY_OBSTACLES

    grid = build_occupancy_grid(obstacles, resolution, robot_radius)

    start_grid = world_to_grid(start_world[0], start_world[1], resolution)
    goal_grid = world_to_grid(goal_world[0], goal_world[1], resolution)

    raw_path = astar(grid, start_grid, goal_grid)
    if raw_path is None:
        return None, grid

    smoothed = smooth_path(raw_path, grid)

    # Convert back to world coordinates
    waypoints = [grid_to_world(c, r, resolution) for c, r in smoothed]

    # Add intermediate waypoints on long segments so proportional steering
    # can make gentle corrections instead of one long blind segment
    waypoints = subdivide_long_segments(waypoints, max_segment_inches=8.0)

    return waypoints, grid


def subdivide_long_segments(waypoints, max_segment_inches=8.0):
    """
    Insert intermediate waypoints along segments longer than max_segment_inches.
    This gives proportional steering more frequent correction points.
    """
    result = [waypoints[0]]
    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        dx, dy = x2 - x1, y2 - y1
        dist = math.sqrt(dx**2 + dy**2)

        if dist > max_segment_inches:
            n_segments = math.ceil(dist / max_segment_inches)
            for j in range(1, n_segments):
                t = j / n_segments
                result.append((x1 + t * dx, y1 + t * dy))

        result.append((x2, y2))
    return result


if __name__ == '__main__':
    ROBOT_RADIUS = 3.0
    RESOLUTION = 4

    start = (40, 32)
    goal = (16, 8)

    print(f"Planning path from {start} to {goal}...")
    waypoints, grid = plan_path(start, goal, RESOLUTION, ROBOT_RADIUS)

    if waypoints:
        print(f"Path found with {len(waypoints)} waypoints:")
        for i, wp in enumerate(waypoints):
            print(f"  {i}: ({wp[0]:.1f}, {wp[1]:.1f})")
        visualize_grid(grid, RESOLUTION, path=waypoints, start=start, goal=goal)
    else:
        print("No path found!")
        visualize_grid(grid, RESOLUTION, start=start, goal=goal)
