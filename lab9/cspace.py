import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap

# ─────────────────────────────────────────────
#  ARM PARAMETERS
# ─────────────────────────────────────────────
L1 = 4.0    # inches, link 1 length
L2 = 4.5    # inches, link 2 length

THETA1_MIN, THETA1_MAX = 0, 180      # degrees
THETA2_MIN, THETA2_MAX = -180, 180   # degrees
STEP = 5                             # degree increment

PADDING = 0.5  # inches of obstacle padding (bloat obstacles by this amount)

# ─────────────────────────────────────────────
#  OBSTACLES  →  edit these for your demo!
#  Each entry: (x_min, y_min, width, height)
#  in workspace inches, origin at arm base
# ─────────────────────────────────────────────
OBSTACLES = [
    (-6, 4, 2, 2),   # left obstacle  (x: -6 to -4, y: 4 to 6)
    ( 2, 5, 3, 2),   # right obstacle (x:  2 to  5, y: 5 to 7)
]

# ─────────────────────────────────────────────
#  GOAL POINTS  (for video submission)
# ─────────────────────────────────────────────
GOAL_POINTS = {
    "A": (3,  2),
    "B": (0,  4),
    "C": (-3, 2),
}


# ─────────────────────────────────────────────
#  FORWARD KINEMATICS
# ─────────────────────────────────────────────
def forward_kinematics(t1_deg, t2_deg):
    """Returns (elbow_xy, ee_xy) given joint angles in degrees."""
    t1 = np.radians(t1_deg)
    t2 = np.radians(t2_deg)
    elbow = np.array([L1 * np.cos(t1),
                      L1 * np.sin(t1)])
    ee    = elbow + np.array([L2 * np.cos(t1 + t2),
                              L2 * np.sin(t1 + t2)])
    return elbow, ee


# ─────────────────────────────────────────────
#  GEOMETRY HELPERS
# ─────────────────────────────────────────────
def cross2d(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def on_segment(p, a, b):
    return (min(a[0],b[0]) <= p[0] <= max(a[0],b[0]) and
            min(a[1],b[1]) <= p[1] <= max(a[1],b[1]))

def segments_intersect(a1, a2, b1, b2):
    d1 = cross2d(b1, b2, a1)
    d2 = cross2d(b1, b2, a2)
    d3 = cross2d(a1, a2, b1)
    d4 = cross2d(a1, a2, b2)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    if d1 == 0 and on_segment(a1, b1, b2): return True
    if d2 == 0 and on_segment(a2, b1, b2): return True
    if d3 == 0 and on_segment(b1, a1, a2): return True
    if d4 == 0 and on_segment(b2, a1, a2): return True
    return False

def point_in_rect(p, rx, ry, rw, rh):
    return rx <= p[0] <= rx+rw and ry <= p[1] <= ry+rh

def segment_hits_rect(p1, p2, rx, ry, rw, rh):
    """True if segment p1->p2 intersects the rectangle (including interior)."""
    if point_in_rect(p1, rx, ry, rw, rh): return True
    if point_in_rect(p2, rx, ry, rw, rh): return True
    corners = [(rx,ry), (rx+rw,ry), (rx+rw,ry+rh), (rx,ry+rh)]
    for i in range(4):
        if segments_intersect(p1, p2, corners[i], corners[(i+1)%4]):
            return True
    return False


# ─────────────────────────────────────────────
#  COLLISION CHECK
# ─────────────────────────────────────────────
def is_collision(t1_deg, t2_deg, obstacles, padding=0.0):
    """Returns True if this configuration collides with any obstacle."""
    elbow, ee = forward_kinematics(t1_deg, t2_deg)
    base = np.array([0.0, 0.0])

    for (ox, oy, ow, oh) in obstacles:
        rx, ry = ox - padding, oy - padding
        rw, rh = ow + 2*padding, oh + 2*padding
        if segment_hits_rect(base,  elbow, rx, ry, rw, rh): return True
        if segment_hits_rect(elbow, ee,    rx, ry, rw, rh): return True
    return False


# ─────────────────────────────────────────────
#  BUILD C-SPACE GRID
# ─────────────────────────────────────────────
def compute_cspace(obstacles, padding=0.0):
    """
    Returns a 2D boolean array.
      axes:  [theta1_index, theta2_index]
      True  = valid (no collision)
      False = invalid (collision or out of bounds)
    """
    t1_vals = np.arange(THETA1_MIN, THETA1_MAX + STEP, STEP)
    t2_vals = np.arange(THETA2_MIN, THETA2_MAX + STEP, STEP)

    grid = np.ones((len(t1_vals), len(t2_vals)), dtype=bool)

    for i, t1 in enumerate(t1_vals):
        for j, t2 in enumerate(t2_vals):
            if is_collision(t1, t2, obstacles, padding):
                grid[i, j] = False

    return grid, t1_vals, t2_vals


# ─────────────────────────────────────────────
#  INVERSE KINEMATICS
# ─────────────────────────────────────────────
def inverse_kinematics(x, y):
    """
    Returns list of (t1, t2) solutions in degrees.
    Filters to only valid joint ranges:
      t1 in [0, 180], t2 in [-180, 180]
    """
    r2 = x**2 + y**2
    cos_t2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(cos_t2) > 1:
        return []

    solutions = []
    for sign in [1, -1]:
        sin_t2 = sign * np.sqrt(1 - cos_t2**2)
        t2 = np.degrees(np.arctan2(sin_t2, cos_t2))
        t1 = np.degrees(np.arctan2(y, x) - np.arctan2(L2*sin_t2, L1 + L2*cos_t2))

        t1 = t1 % 360
        t2 = (t2 + 180) % 360 - 180

        if THETA1_MIN <= t1 <= THETA1_MAX and THETA2_MIN <= t2 <= THETA2_MAX:
            solutions.append((round(t1, 2), round(t2, 2)))

    return solutions


# ─────────────────────────────────────────────
#  WAYPOINTS  (start + goal points)
# ─────────────────────────────────────────────
START_POINT = (6.25, 0)

WAYPOINTS = {
    "Start": START_POINT,
    "A":     GOAL_POINTS["A"],
    "B":     GOAL_POINTS["B"],
    "C":     GOAL_POINTS["C"],
}


def best_ik_solution(x, y):
    """
    Returns the best valid (non-colliding) IK solution for (x, y).
    Prefers collision-free; falls back to first solution if none are free.
    Returns (t1, t2) or None if unreachable.
    """
    solutions = inverse_kinematics(x, y)
    if not solutions:
        return None
    for t1, t2 in solutions:
        if not is_collision(t1, t2, OBSTACLES, PADDING):
            return (t1, t2)
    return solutions[0]


def draw_workspace_panel(ax, t1, t2, label, target_xy):
    """Draw one workspace panel showing the arm at the given configuration."""
    ax.set_facecolor("white")
    ax.set_xlim(-7, 7)
    ax.set_ylim(0, 8)
    ax.set_aspect("equal")

    for x in range(-7, 8):
        ax.axvline(x, color="#cccccc", linewidth=0.5, zorder=0)
    for y in range(0, 9):
        ax.axhline(y, color="#cccccc", linewidth=0.5, zorder=0)

    ax.set_xticks(range(-7, 8, 1))
    ax.set_yticks(range(0, 9, 1))
    ax.set_xticklabels([str(x) if x % 2 != 0 else "" for x in range(-7, 8)], fontsize=6)
    ax.set_yticklabels([str(y) if y % 2 != 0 else "" for y in range(0, 9)], fontsize=6)
    ax.tick_params(length=0)

    for (ox, oy, ow, oh) in OBSTACLES:
        ax.add_patch(patches.Rectangle(
            (ox, oy), ow, oh,
            facecolor="#e74c3c", edgecolor="#c0392b", linewidth=1.5, zorder=2
        ))

    ax.plot(*target_xy, "X", color="#f1c40f", markersize=11, zorder=6, markeredgewidth=2)

    elbow, ee = forward_kinematics(t1, t2)
    base = np.array([0.0, 0.0])
    ax.plot([base[0], elbow[0]], [base[1], elbow[1]],
            color="black", linewidth=2.5, zorder=3, solid_capstyle="round")
    ax.plot([elbow[0], ee[0]], [elbow[1], ee[1]],
            color="black", linewidth=2.5, zorder=3, solid_capstyle="round")

    ax.plot(*base, "o", color="#3498db", markersize=9, zorder=5)
    ax.plot(*ee,   "o", color="#8e44ad", markersize=7, zorder=5)

    for spine in ax.spines.values():
        spine.set_edgecolor("#27ae60")
        spine.set_linewidth(2)

    gx, gy = target_xy
    ax.set_title(f"{label}:  ({gx}, {gy})\ntheta1={t1:.1f}  theta2={t2:.1f}",
                 fontsize=9, pad=6, fontweight="bold")


def plot_example_orientations():
    """4-panel figure: arm at Start, A, B, C using IK."""
    fig, axes = plt.subplots(2, 2, figsize=(11, 9))
    fig.patch.set_facecolor("white")
    fig.suptitle("Arm Configurations: Start -> A -> B -> C",
                 fontsize=13, fontweight="bold")

    print("\nIK solutions for orientation panels:")
    for ax, (label, (gx, gy)) in zip(axes.flat, WAYPOINTS.items()):
        sol = best_ik_solution(gx, gy)
        if sol is None:
            print(f"  {label} = ({gx}, {gy}): UNREACHABLE")
            ax.set_title(f"{label} ({gx},{gy}) - UNREACHABLE", color="red", fontsize=9)
            continue
        t1, t2 = sol
        col = is_collision(t1, t2, OBSTACLES, PADDING)
        print(f"  {label} = ({gx}, {gy}):  t1={t1:.2f}  t2={t2:.2f}"
              f"  [{'COLLISION' if col else 'OK'}]")
        draw_workspace_panel(ax, t1, t2, label, target_xy=(gx, gy))

    plt.tight_layout()
    plt.savefig("example_orientations.png", dpi=150, bbox_inches="tight", facecolor="white")
    plt.show()
    print("Saved: example_orientations.png")


# ─────────────────────────────────────────────
#  PLOTTING
# ─────────────────────────────────────────────
def plot_cspace(grid, t1_vals, t2_vals, goal_ik_points=None):
    """Plot the C-space with obstacles marked and goal points highlighted."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.patch.set_facecolor("#1a1a2e")

    ax = axes[0]
    ax.set_facecolor("#16213e")

    cmap = ListedColormap(["#8b0000", "#2ecc71"])
    ax.imshow(
        grid.T,
        origin="lower",
        extent=[t1_vals[0]-STEP/2, t1_vals[-1]+STEP/2,
                t2_vals[0]-STEP/2, t2_vals[-1]+STEP/2],
        cmap=cmap, vmin=0, vmax=1,
        aspect="auto", interpolation="nearest"
    )

    if goal_ik_points:
        colors = {"A": "#f39c12", "B": "#e74c3c", "C": "#3498db"}
        for label, solutions in goal_ik_points.items():
            for t1, t2 in solutions:
                ax.plot(t1, t2, "o", color=colors.get(label, "white"),
                        markersize=10, zorder=5)
                ax.annotate(label, (t1, t2), textcoords="offset points",
                            xytext=(6, 4), color=colors.get(label, "white"),
                            fontsize=10, fontweight="bold")

    ax.set_xlabel("theta1 (degrees)", color="white", fontsize=11)
    ax.set_ylabel("theta2 (degrees)", color="white", fontsize=11)
    ax.set_title("Configuration Space (C-Space)", color="white", fontsize=13, fontweight="bold")
    ax.set_xlim(THETA1_MIN, THETA1_MAX)
    ax.set_ylim(THETA2_MIN, THETA2_MAX)
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_edgecolor("#444")

    from matplotlib.patches import Patch
    legend = [Patch(facecolor="#2ecc71", label="Valid"),
              Patch(facecolor="#8b0000", label="Collision")]
    ax.legend(handles=legend, loc="upper right",
              facecolor="#1a1a2e", labelcolor="white", fontsize=9)

    ax2 = axes[1]
    ax2.set_facecolor("#16213e")
    ax2.set_aspect("equal")
    ax2.set_xlim(-8, 8)
    ax2.set_ylim(-1, 9)
    ax2.axhline(0, color="#444", linewidth=0.8)
    ax2.axvline(0, color="#444", linewidth=0.8)
    ax2.grid(True, color="#2a2a4a", linewidth=0.5)

    for (ox, oy, ow, oh) in OBSTACLES:
        ax2.add_patch(patches.Rectangle((ox, oy), ow, oh,
                      facecolor="#c0392b", edgecolor="#e74c3c",
                      linewidth=1.5, alpha=0.85))
        ax2.add_patch(patches.Rectangle((ox-PADDING, oy-PADDING),
                      ow+2*PADDING, oh+2*PADDING,
                      facecolor="none", edgecolor="#e74c3c",
                      linewidth=1, linestyle="--", alpha=0.5))

    sample_t1, sample_t2 = 90, 0
    elbow, ee = forward_kinematics(sample_t1, sample_t2)
    base = [0, 0]
    ax2.plot([base[0], elbow[0]], [base[1], elbow[1]], "w-", linewidth=3, zorder=3)
    ax2.plot([elbow[0], ee[0]], [elbow[1], ee[1]], color="#aaa", linewidth=3, zorder=3)
    ax2.plot(*base,  "o", color="#3498db", markersize=10, zorder=4)
    ax2.plot(*elbow, "o", color="white",   markersize=7,  zorder=4)
    ax2.plot(*ee,    "*", color="#f1c40f", markersize=12, zorder=4)

    gcolors = {"A": "#f39c12", "B": "#e74c3c", "C": "#3498db"}
    for label, (gx, gy) in GOAL_POINTS.items():
        ax2.plot(gx, gy, "X", color=gcolors[label], markersize=12, zorder=5)
        ax2.annotate(label, (gx, gy), textcoords="offset points",
                     xytext=(6, 4), color=gcolors[label],
                     fontsize=11, fontweight="bold")

    ax2.set_xlabel("X (inches)", color="white", fontsize=11)
    ax2.set_ylabel("Y (inches)", color="white", fontsize=11)
    ax2.set_title("Workspace", color="white", fontsize=13, fontweight="bold")
    ax2.tick_params(colors="white")
    for spine in ax2.spines.values():
        spine.set_edgecolor("#444")

    plt.tight_layout()
    plt.savefig("cspace.png", dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.show()
    print("Saved: cspace.png")


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────
if __name__ == "__main__":
    print("Computing C-space...")
    grid, t1_vals, t2_vals = compute_cspace(OBSTACLES, padding=PADDING)

    total = grid.size
    valid = grid.sum()
    print(f"  Grid size : {grid.shape[0]} x {grid.shape[1]} = {total} configs")
    print(f"  Valid     : {valid}  ({100*valid/total:.1f}%)")
    print(f"  Collision : {total-valid}  ({100*(total-valid)/total:.1f}%)")

    print("\nInverse Kinematics for goal points:")
    goal_ik = {}
    for label, (gx, gy) in GOAL_POINTS.items():
        sols = inverse_kinematics(gx, gy)
        goal_ik[label] = sols
        print(f"  {label} = ({gx}, {gy}):")
        if sols:
            for t1, t2 in sols:
                col = is_collision(t1, t2, OBSTACLES, PADDING)
                status = "COLLISION" if col else "OK"
                print(f"    t1={t1:7.2f}  t2={t2:7.2f} -> {status}")
        else:
            print("    UNREACHABLE")

    plot_cspace(grid, t1_vals, t2_vals, goal_ik_points=goal_ik)

    print("\nArm orientation configs:")
    plot_example_orientations()