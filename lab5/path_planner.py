import heapq
import math
import numpy as np
from map_grid import (
    build_occupancy_grid, world_to_grid, grid_to_world,
    visualize_grid, Easy_Obstacles, Robot_Radius, Resolution,
    Map_Width, Map_Height
)

def astar(grid, start_grid, goal_grid):
    
    rows, cols = grid.shape
    sc, sr = start_grid
    gc, gr = goal_grid

    # edge caes if its tight but not impossible: warn but proceed
    if grid[sr, sc] or grid[gr, gc]:
        print("ERROR: Start or goal is in c-space - try reducing Robot_Radius or adjust position")
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

#Bresenham
def line_of_sight(grid, p1, p2):
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
    if obstacles is None:
        obstacles = Easy_Obstacles

    inflated_grid, original_grid = build_occupancy_grid(obstacles, resolution, robot_radius)

    # Validate that start and goal are within map bounds
    sx, sy = start_world
    gx, gy = goal_world

    if not (0 <= sx < Map_Width and 0 <= sy < Map_Height):
        print(f"ERROR: Start position ({sx}, {sy}) is outside map bounds!")
        print(f"Map size: {Map_Width}\" × {Map_Height}\" (0,0 to {Map_Width},{Map_Height})")
        return None, (inflated_grid, original_grid)

    if not (0 <= gx < Map_Width and 0 <= gy < Map_Height):
        print(f"ERROR: Goal position ({gx}, {gy}) is outside map bounds!")
        print(f"Map size: {Map_Width}\" × {Map_Height}\" (0,0 to {Map_Width},{Map_Height})")
        return None, (inflated_grid, original_grid)

    start_grid = world_to_grid(sx, sy, resolution)
    goal_grid = world_to_grid(gx, gy, resolution)

    # Smart validation: distinguish between c-space and real obstacle collisions
    sc, sr = start_grid
    gc, gr = goal_grid

    # Check if start is in a REAL obstacle (hard error)
    if original_grid[sr, sc]:
        print(f"ERROR: Start position {start_world} is inside a real obstacle!")
        return None, (inflated_grid, original_grid)

    # Check if goal is in a REAL obstacle (hard error)
    if original_grid[gr, gc]:
        print(f"ERROR: Goal position {goal_world} is inside a real obstacle!")
        return None, (inflated_grid, original_grid)

    # Check if start/goal is in c-space but not real obstacle (warning, but proceed)
    if inflated_grid[sr, sc]:
        print(f"WARNING: Start {start_world} is in c-space inflation zone - path may be tight!")
    if inflated_grid[gr, gc]:
        print(f"WARNING: Goal {goal_world} is in c-space inflation zone - path may be tight!")

    raw_path = astar(inflated_grid, start_grid, goal_grid)
    if raw_path is None:
        return None, (inflated_grid, original_grid)

    smoothed = smooth_path(raw_path, inflated_grid)

    # Convert back to world coordinates
    waypoints = [grid_to_world(c, r, resolution) for c, r in smoothed]

    # Add intermediate waypoints on long segments so proportional steering
    # can make gentle corrections instead of one long blind segment
    waypoints = subdivide_long_segments(waypoints, max_segment_inches=8.0)

    return waypoints, (inflated_grid, original_grid)


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
    # Test path planning with values from map_grid.py
    start = (6, 6)
    goal = (40, 47)

    print(f"Planning path from {start} to {goal}...")
    print(f"Using Robot_Radius={Robot_Radius}, Resolution={Resolution}")

    waypoints, grids = plan_path(start, goal, Resolution, Robot_Radius)
    inflated_grid, original_grid = grids

    if waypoints:
        print(f"Path found with {len(waypoints)} waypoints:")
        for i, wp in enumerate(waypoints):
            print(f"  {i}: ({wp[0]:.1f}, {wp[1]:.1f})")
        visualize_grid(inflated_grid, original_grid, path=waypoints, start=start, goal=goal)
    else:
        print("No path found!")
        visualize_grid(inflated_grid, original_grid, start=start, goal=goal)

    for i,wp in enumerate(waypoints):
        print((wp[0], wp[1]),",", end=" ")
