"""
16-281 Lab 9 — Robot Arm Path Planner  (with diagnostics)
Two-link RR arm: L1=4in, L2=4.5in
Motors: CH_JOINT1 = θ1 (base), CH_JOINT2 = θ2 (elbow)
Horizontal orientation — no gravity compensation.

Visualization windows:
  Figure 1 — Pre-run:  C-space map + workspace trajectory (confirm plan looks sane)
  Figure 2 — Live run: workspace arm animation + commanded vs actual joint angles
  Figure 3 — Post-run: error summary per waypoint (mechanical vs software diagnosis)

Run with:  python arm_path_planner.py
           python arm_path_planner.py --dry-run   (plan + visualize, no motors)
"""

import sys
import math
import time
import heapq
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D

# ─── Try importing motor library ─────────────────────────────────────────────
DRY_RUN = "--dry-run" in sys.argv
try:
    from motorgo import Plink, ControlMode
except ImportError:
    print("[WARN] motorgo not found — forcing --dry-run mode")
    DRY_RUN = True

# ─── Arm geometry ─────────────────────────────────────────────────────────────
L1 = 4.0
L2 = 4.5

# ─── Motor setup ──────────────────────────────────────────────────────────────
CH_JOINT1 = 1      # θ1  base
CH_JOINT2 = 2      # θ2  elbow
DIR_J1    = 1      # flip to -1 if joint moves backwards
DIR_J2    = 1

# ─── C-space grid ─────────────────────────────────────────────────────────────
RES    = 2
T1_MIN, T1_MAX = 0, 180
T2_MIN, T2_MAX = -180, 180
T1N = int((T1_MAX - T1_MIN) / RES) + 1   # 91
T2N = int((T2_MAX - T2_MIN) / RES) + 1   # 181

# ─── Obstacles  [x_min, x_max, y_min, y_max]  (inches) ───────────────────────
OBSTACLES = [
    [-4.2, -1.8, 3.8, 6.2],
    [ 0.8,  3.2, 4.8, 7.2],
]
PAD = 0.0   # obstacle coords already include padding

# ─── Waypoints (inches) ───────────────────────────────────────────────────────
START = (6.25, 0.0)
A     = (3.0,  2.0)
B     = (0.0,  4.0)
C     = (-3.0, 2.0)

WAYPOINTS      = [START, A, B, C]
WAYPOINT_NAMES = ["Start", "A", "B", "C"]
WP_COLORS      = ["#3266ad", "#1D9E75", "#EF9F27", "#D4537E"]

# ─── Execution ────────────────────────────────────────────────────────────────
HOLD_SEC      = 3.0
STEP_SLEEP    = 0.05
POS_TOLERANCE = 3.0     # degrees


# ══════════════════════════════════════════════════════════════════════════════
# Kinematics
# ══════════════════════════════════════════════════════════════════════════════

def fk(t1d, t2d):
    t1, t2 = math.radians(t1d), math.radians(t2d)
    jx = L1 * math.cos(t1);  jy = L1 * math.sin(t1)
    ex = jx + L2 * math.cos(t1 + t2)
    ey = jy + L2 * math.sin(t1 + t2)
    return jx, jy, ex, ey

def ik(x, y):
    r2  = x*x + y*y
    c2  = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c2) > 1.0:
        return []
    sols = []
    for sign in [1, -1]:
        t2r = math.atan2(sign * math.sqrt(max(0, 1 - c2**2)), c2)
        t1r = math.atan2(y, x) - math.atan2(L2 * math.sin(t2r),
                                              L1 + L2 * math.cos(t2r))
        t1d = math.degrees(t1r) % 360
        if 0 <= t1d <= 180:
            sols.append((t1d, math.degrees(t2r)))
    return sols


# ══════════════════════════════════════════════════════════════════════════════
# C-space
# ══════════════════════════════════════════════════════════════════════════════

def _seg_hits(ax, ay, bx, by, x1, x2, y1, y2, pad, n=25):
    for k in range(n + 1):
        t  = k / n
        px = ax + t * (bx - ax);  py = ay + t * (by - ay)
        if x1-pad <= px <= x2+pad and y1-pad <= py <= y2+pad:
            return True
    return False

def _collides(t1d, t2d, obs, pad):
    jx, jy, ex, ey = fk(t1d, t2d)
    for (x1, x2, y1, y2) in obs:
        if _seg_hits(0, 0, jx, jy, x1, x2, y1, y2, pad): return True
        if _seg_hits(jx, jy, ex, ey, x1, x2, y1, y2, pad): return True
    return False

def build_cspace(obs, pad):
    print("Building C-space...", end=" ", flush=True)
    cm = np.zeros((T1N, T2N), dtype=np.uint8)
    for i in range(T1N):
        for j in range(T2N):
            t1, t2 = T1_MIN + i*RES, T2_MIN + j*RES
            if _collides(t1, t2, obs, 0):    cm[i, j] = 1
            elif _collides(t1, t2, obs, pad): cm[i, j] = 2
    print("done.")
    return cm

def cell_of(t1d, t2d):
    i = max(0, min(T1N-1, round((t1d - T1_MIN) / RES)))
    j = round((t2d - T2_MIN) / RES) % T2N
    return i, j

def angles_of(i, j):
    return T1_MIN + i*RES, T2_MIN + j*RES


# ══════════════════════════════════════════════════════════════════════════════
# A*
# ══════════════════════════════════════════════════════════════════════════════

_DIRS = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]

def _h(i, j, gi, gj):
    di = abs(i - gi) * RES
    dj = min(abs(j - gj), T2N - abs(j - gj)) * RES
    return math.hypot(di, dj)

def astar(si, sj, gi, gj, cmap):
    INF  = float('inf')
    dist = np.full((T1N, T2N), INF, np.float32)
    prev = np.full((T1N, T2N, 2), -1, np.int16)
    dist[si, sj] = 0.0
    # heap stores (f, g, i, j) — g kept separately so stale check compares g vs dist
    heap = [(_h(si, sj, gi, gj), 0.0, si, sj)]
    while heap:
        f, g, ci, cj = heapq.heappop(heap)
        if ci == gi and cj == gj:
            path = []
            ni, nj = ci, cj
            while ni >= 0:
                path.append((ni, nj))
                pi, pj = prev[ni, nj]
                ni, nj = int(pi), int(pj)
            return path[::-1]
        if g > dist[ci, cj] + 1e-4: continue   # stale entry — skip
        for di, dj in _DIRS:
            ni = ci + di;  nj = (cj + dj + T2N) % T2N
            if ni < 0 or ni >= T1N: continue
            v  = cmap[ni, nj]
            if v == 1: continue
            step = math.hypot(di * RES, dj * RES) + (40.0 if v == 2 else 0.0)
            nd   = dist[ci, cj] + step
            if nd < dist[ni, nj]:
                dist[ni, nj] = nd;  prev[ni, nj] = (ci, cj)
                heapq.heappush(heap, (nd + _h(ni, nj, gi, gj), nd, ni, nj))
    return None

def plan_segment(from_xy, to_xy, cmap, label=""):
    from_sols = [s for s in ik(*from_xy) if cmap[cell_of(*s)] != 1]
    to_sols   = [s for s in ik(*to_xy)   if cmap[cell_of(*s)] != 1]
    if not from_sols: raise ValueError(f"No free IK for {label} start {from_xy}")
    if not to_sols:   raise ValueError(f"No free IK for {label} goal  {to_xy}")
    best, best_len = None, float('inf')
    for fs in from_sols:
        si, sj = cell_of(*fs)
        for ts in to_sols:
            gi, gj = cell_of(*ts)
            p = astar(si, sj, gi, gj, cmap)
            if p and len(p) < best_len:
                best_len = len(p);  best = p
    if best is None: raise RuntimeError(f"No path found to {label}")
    return [angles_of(i, j) for i, j in best]

def downsample(path, delta=4.0):
    if len(path) <= 2: return path
    out = [path[0]]
    for pt in path[1:-1]:
        d1 = abs(pt[0] - out[-1][0])
        d2 = min(abs(pt[1] - out[-1][1]), 360 - abs(pt[1] - out[-1][1]))
        if d1 >= delta or d2 >= delta: out.append(pt)
    out.append(path[-1])
    return out


# ══════════════════════════════════════════════════════════════════════════════
# Figure 1 — Pre-run: C-space + workspace plan
# ══════════════════════════════════════════════════════════════════════════════

def show_plan(cmap, full_path, named_configs):
    fig = plt.figure("Pre-run: Planned trajectory", figsize=(13, 5))
    fig.patch.set_facecolor("#1a1a1a")
    gs  = GridSpec(1, 2, figure=fig, wspace=0.08)
    ax_cs = fig.add_subplot(gs[0])
    ax_ws = fig.add_subplot(gs[1])
    for ax in [ax_cs, ax_ws]:
        ax.set_facecolor("#111")
        for sp in ax.spines.values(): sp.set_color("#444")
        ax.tick_params(colors="#888", labelsize=8)

    # C-space image
    img = np.zeros((*cmap.T.shape, 4))  # transposed: x=T2, y=T1
    img[cmap.T == 0] = [0.08, 0.22, 0.08, 1.0]
    img[cmap.T == 1] = [0.85, 0.28, 0.28, 1.0]
    img[cmap.T == 2] = [0.85, 0.45, 0.25, 0.45]
    ax_cs.imshow(img, origin="lower",
                 extent=[T1_MIN, T1_MAX, T2_MIN, T2_MAX],
                 aspect="auto", interpolation="nearest")

    # Path line on C-space  (x-axis = θ1, y-axis = θ2)
    t1s = [t1 for (t1, t2) in full_path]
    t2s = [t2 for (t1, t2) in full_path]
    ax_cs.plot(t1s, t2s, color="white", lw=1.2, alpha=0.85, zorder=3)

    for name, color, t1, t2 in named_configs:
        ax_cs.plot(t1, t2, "o", color=color, ms=8, zorder=5,
                   markeredgecolor="white", markeredgewidth=0.8)
        ax_cs.text(t1 + 2, t2 + 3, name, color=color, fontsize=8, zorder=6)

    ax_cs.set_xlabel("θ₁ (deg)", color="#888", fontsize=9)
    ax_cs.set_ylabel("θ₂ (deg)", color="#888", fontsize=9)
    ax_cs.set_title("C-space  (white = planned path)", color="#ccc", fontsize=10)
    ax_cs.grid(color="#333", lw=0.4)

    # Workspace
    ax_ws.set_xlim(-8, 8);  ax_ws.set_ylim(-1, 9)
    ax_ws.set_aspect("equal");  ax_ws.grid(color="#333", lw=0.4)
    ax_ws.axhline(0, color="#555", lw=0.6);  ax_ws.axvline(0, color="#555", lw=0.6)
    ax_ws.set_xlabel("x (in)", color="#888", fontsize=9)
    ax_ws.set_ylabel("y (in)", color="#888", fontsize=9)
    ax_ws.set_title("Workspace trajectory", color="#ccc", fontsize=10)

    for (x1, x2, y1, y2) in OBSTACLES:
        ax_ws.add_patch(patches.Rectangle((x1, y1), x2-x1, y2-y1,
                        fc="#e24b4a", alpha=0.55, ec="#e24b4a", lw=0.8))
        ax_ws.add_patch(patches.Rectangle((x1-PAD, y1-PAD),
                        x2-x1+2*PAD, y2-y1+2*PAD,
                        fc="none", ec="#e8593c", lw=0.6, ls="--", alpha=0.5))

    seg_colors = [WP_COLORS[1], WP_COLORS[2], WP_COLORS[3]]
    seg_ends   = [cell_of(*wp) for wp in WAYPOINTS[1:]]
    seg_idx, ptr = 0, 0
    sx, sy = [], []
    while ptr < len(full_path) and seg_idx < 3:
        t1, t2 = full_path[ptr]
        _, _, ex, ey = fk(t1, t2)
        sx.append(ex);  sy.append(ey)
        ci, cj = cell_of(t1, t2)
        ptr += 1
        if (ci, cj) == seg_ends[seg_idx]:
            ax_ws.plot(sx, sy, color=seg_colors[seg_idx], lw=1.5, alpha=0.8)
            sx, sy = [ex], [ey]
            seg_idx += 1

    for i, (wx, wy) in enumerate(WAYPOINTS):
        ax_ws.plot(wx, wy, "o", color=WP_COLORS[i], ms=9,
                   markeredgecolor="white", markeredgewidth=0.8, zorder=6)
        ax_ws.text(wx + 0.15, wy + 0.15, WAYPOINT_NAMES[i],
                   color=WP_COLORS[i], fontsize=9, fontweight="bold")

    legend_els = [
        Line2D([0],[0], marker="s", color="w", markerfacecolor="#e24b4a", ms=8, label="Obstacle"),
        Line2D([0],[0], color=WP_COLORS[1], lw=2, label="S→A"),
        Line2D([0],[0], color=WP_COLORS[2], lw=2, label="A→B"),
        Line2D([0],[0], color=WP_COLORS[3], lw=2, label="B→C"),
    ]
    ax_ws.legend(handles=legend_els, loc="lower right",
                 facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc", fontsize=8)

    plt.tight_layout()
    plt.show(block=False);  plt.pause(0.1)
    return fig


# ══════════════════════════════════════════════════════════════════════════════
# Figure 2 — Live execution monitor
# ══════════════════════════════════════════════════════════════════════════════

class LiveMonitor:
    def __init__(self):
        self.fig = plt.figure("Live execution", figsize=(13, 5))
        self.fig.patch.set_facecolor("#1a1a1a")
        gs = GridSpec(2, 2, figure=self.fig, wspace=0.12, hspace=0.38)

        self.ax_ws = self.fig.add_subplot(gs[:, 0])
        self.ax_t1 = self.fig.add_subplot(gs[0, 1])
        self.ax_t2 = self.fig.add_subplot(gs[1, 1])

        for ax in [self.ax_ws, self.ax_t1, self.ax_t2]:
            ax.set_facecolor("#111")
            for sp in ax.spines.values(): sp.set_color("#444")
            ax.tick_params(colors="#888", labelsize=8)

        # Workspace
        ax = self.ax_ws
        ax.set_xlim(-8, 8);  ax.set_ylim(-1, 9)
        ax.set_aspect("equal");  ax.grid(color="#333", lw=0.4)
        ax.axhline(0, color="#555", lw=0.6);  ax.axvline(0, color="#555", lw=0.6)
        ax.set_xlabel("x (in)", color="#888", fontsize=9)
        ax.set_ylabel("y (in)", color="#888", fontsize=9)
        ax.set_title("Commanded (solid blue/green)  vs  Actual (dashed orange)",
                     color="#ccc", fontsize=9)

        for (x1,x2,y1,y2) in OBSTACLES:
            ax.add_patch(patches.Rectangle((x1,y1), x2-x1, y2-y1,
                         fc="#e24b4a", alpha=0.45, ec="#e24b4a", lw=0.8))
        for i,(wx,wy) in enumerate(WAYPOINTS):
            ax.plot(wx,wy,"o",color=WP_COLORS[i],ms=7,
                    markeredgecolor="white",markeredgewidth=0.8,zorder=6)
            ax.text(wx+0.15,wy+0.15,WAYPOINT_NAMES[i],
                    color=WP_COLORS[i],fontsize=9,fontweight="bold")

        self.cmd_l1, = ax.plot([], [], color="#3266ad", lw=3, solid_capstyle="round")
        self.cmd_l2, = ax.plot([], [], color="#1D9E75", lw=3, solid_capstyle="round")
        self.act_l1, = ax.plot([], [], color="#f4a023", lw=2, ls="--", alpha=0.85)
        self.act_l2, = ax.plot([], [], color="#f4a023", lw=2, ls="--", alpha=0.85)
        self.cmd_ee, = ax.plot([], [], "o", color="white",   ms=6, zorder=8)
        self.act_ee, = ax.plot([], [], "o", color="#f4a023", ms=6, zorder=8)
        self.trace_x, self.trace_y = [], []
        self.trace,  = ax.plot([], [], color="white", lw=0.8, alpha=0.35)

        legend_els = [
            Line2D([0],[0],color="#3266ad",lw=2,label="Commanded"),
            Line2D([0],[0],color="#f4a023",lw=2,ls="--",label="Actual"),
        ]
        ax.legend(handles=legend_els, loc="lower right",
                  facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc", fontsize=8)

        # θ1 subplot
        self.ax_t1.set_ylabel("θ₁ (deg)", color="#888", fontsize=8)
        self.ax_t1.set_title("Joint angles — commanded vs actual", color="#ccc", fontsize=9)
        self.ax_t1.grid(color="#333", lw=0.4)
        self.t1c, = self.ax_t1.plot([], [], color="#3266ad", lw=1.2, label="cmd θ₁")
        self.t1a, = self.ax_t1.plot([], [], color="#f4a023", lw=1.2, ls="--", label="act θ₁")
        self.ax_t1.legend(loc="upper right", facecolor="#1a1a1a",
                          edgecolor="#444", labelcolor="#ccc", fontsize=7)

        # θ2 subplot
        self.ax_t2.set_xlabel("time (s)", color="#888", fontsize=8)
        self.ax_t2.set_ylabel("θ₂ (deg)", color="#888", fontsize=8)
        self.ax_t2.grid(color="#333", lw=0.4)
        self.t2c, = self.ax_t2.plot([], [], color="#1D9E75", lw=1.2, label="cmd θ₂")
        self.t2a, = self.ax_t2.plot([], [], color="#f4a023", lw=1.2, ls="--", label="act θ₂")
        self.ax_t2.legend(loc="upper right", facecolor="#1a1a1a",
                          edgecolor="#444", labelcolor="#ccc", fontsize=7)

        self.times  = []
        self.t1cs   = []; self.t1as = []
        self.t2cs   = []; self.t2as = []
        self.t0     = time.time()

        plt.tight_layout();  plt.show(block=False);  plt.pause(0.05)

    def update(self, t1_cmd, t2_cmd, t1_act, t2_act):
        now = time.time() - self.t0
        self.times.append(now)
        self.t1cs.append(t1_cmd); self.t1as.append(t1_act)
        self.t2cs.append(t2_cmd); self.t2as.append(t2_act)

        jx, jy, ex, ey = fk(t1_cmd, t2_cmd)
        self.cmd_l1.set_data([0, jx], [0, jy])
        self.cmd_l2.set_data([jx, ex], [jy, ey])
        self.cmd_ee.set_data([ex], [ey])
        self.trace_x.append(ex); self.trace_y.append(ey)
        self.trace.set_data(self.trace_x, self.trace_y)

        ajx, ajy, aex, aey = fk(t1_act, t2_act)
        self.act_l1.set_data([0, ajx], [0, ajy])
        self.act_l2.set_data([ajx, aex], [ajy, aey])
        self.act_ee.set_data([aex], [aey])

        ts = self.times
        self.t1c.set_data(ts, self.t1cs);  self.t1a.set_data(ts, self.t1as)
        self.t2c.set_data(ts, self.t2cs);  self.t2a.set_data(ts, self.t2as)
        for ax in [self.ax_t1, self.ax_t2]:
            ax.relim(); ax.autoscale_view()

        self.fig.canvas.flush_events();  plt.pause(0.001)

    def mark_waypoint(self, name, color):
        t = self.times[-1] if self.times else 0
        for ax in [self.ax_t1, self.ax_t2]:
            ax.axvline(t, color=color, lw=0.8, ls=":", alpha=0.7)
            ax.text(t, ax.get_ylim()[0], name, color=color,
                    fontsize=7, rotation=90, va="bottom")
        self.fig.canvas.flush_events();  plt.pause(0.001)


# ══════════════════════════════════════════════════════════════════════════════
# Figure 3 — Post-run error summary
# ══════════════════════════════════════════════════════════════════════════════

def show_error_summary(records):
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    fig.patch.set_facecolor("#1a1a1a")
    fig.suptitle(
        "Post-run diagnostics\n"
        "  Large joint error + small EE error  →  mechanical backlash / flex\n"
        "  Small joint error + large EE error  →  IK / calibration offset\n"
        "  Both large                           →  motor not reaching target (tune PID / slow down)",
        color="#aaa", fontsize=9, x=0.01, ha="left"
    )

    names  = [r["name"]     for r in records]
    colors = [r["color"]    for r in records]
    ee_err = [r["ee_error"] for r in records]
    t1_err = [abs(r["t1_act"] - r["t1_cmd"]) for r in records]
    t2_err = [abs(r["t2_act"] - r["t2_cmd"]) for r in records]

    for ax in axes:
        ax.set_facecolor("#111")
        for sp in ax.spines.values(): sp.set_color("#444")
        ax.tick_params(colors="#888", labelsize=8)

    # EE error
    bars = axes[0].bar(names, ee_err, color=colors, edgecolor="#444", width=0.5)
    axes[0].axhline(0.5, color="#1D9E75", lw=1.2, ls="--", label="0.5\" (full pts)")
    axes[0].axhline(1.0, color="#EF9F27", lw=1.2, ls="--", label="1.0\" (half pts)")
    axes[0].set_ylabel("EE error (in)", color="#888", fontsize=9)
    axes[0].set_title("End-effector error", color="#ccc", fontsize=10)
    axes[0].legend(facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc", fontsize=8)
    for bar, v in zip(bars, ee_err):
        axes[0].text(bar.get_x() + bar.get_width()/2, v + 0.01,
                     f'{v:.3f}"', ha="center", color="#ccc", fontsize=8)

    # θ1 error
    b1 = axes[1].bar(names, t1_err, color=colors, edgecolor="#444", width=0.5)
    axes[1].axhline(POS_TOLERANCE, color="#f4a023", lw=1.2, ls="--",
                    label=f"{POS_TOLERANCE}° tol")
    axes[1].set_ylabel("θ₁ error (deg)", color="#888", fontsize=9)
    axes[1].set_title("Joint 1 error", color="#ccc", fontsize=10)
    axes[1].legend(facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc", fontsize=8)
    for bar, v in zip(b1, t1_err):
        axes[1].text(bar.get_x() + bar.get_width()/2, v + 0.05,
                     f"{v:.1f}°", ha="center", color="#ccc", fontsize=8)

    # θ2 error
    b2 = axes[2].bar(names, t2_err, color=colors, edgecolor="#444", width=0.5)
    axes[2].axhline(POS_TOLERANCE, color="#f4a023", lw=1.2, ls="--",
                    label=f"{POS_TOLERANCE}° tol")
    axes[2].set_ylabel("θ₂ error (deg)", color="#888", fontsize=9)
    axes[2].set_title("Joint 2 error", color="#ccc", fontsize=10)
    axes[2].legend(facecolor="#1a1a1a", edgecolor="#444", labelcolor="#ccc", fontsize=8)
    for bar, v in zip(b2, t2_err):
        axes[2].text(bar.get_x() + bar.get_width()/2, v + 0.05,
                     f"{v:.1f}°", ha="center", color="#ccc", fontsize=8)

    plt.tight_layout(rect=[0, 0, 1, 0.78])
    plt.show(block=False);  plt.pause(0.1)
    input("\nPress Enter to close and exit.\n")
    return fig


# ══════════════════════════════════════════════════════════════════════════════
# Motor helpers
# ══════════════════════════════════════════════════════════════════════════════

def move_to(j1, j2, t1, t2):
    j1.position_command = DIR_J1 * t1
    j2.position_command = DIR_J2 * t2

def read_actual(j1, j2):
    return j1.position / DIR_J1, j2.position / DIR_J2

def wait_arrive(j1, j2, t1_tgt, t2_tgt, monitor=None, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        t1a, t2a = read_actual(j1, j2)
        if monitor:
            monitor.update(t1_tgt, t2_tgt, t1a, t2a)
        if abs(t1a - t1_tgt) < POS_TOLERANCE and abs(t2a - t2_tgt) < POS_TOLERANCE:
            return True
        time.sleep(0.02)
    return False


def execute_segment(j1, j2, path, wp_name, wp_color, monitor, error_records):
    print(f"  → {wp_name} ({len(path)} waypoints)")
    for t1, t2 in path:
        move_to(j1, j2, t1, t2)
        t1a, t2a = read_actual(j1, j2)
        if monitor:
            monitor.update(t1, t2, t1a, t2a)
        time.sleep(STEP_SLEEP)

    t1_goal, t2_goal = path[-1]
    wait_arrive(j1, j2, t1_goal, t2_goal, monitor, timeout=8.0)

    if monitor:
        monitor.mark_waypoint(wp_name, wp_color)

    t1_act, t2_act = read_actual(j1, j2)
    _, _, ex_cmd, ey_cmd = fk(t1_goal, t2_goal)
    _, _, ex_act, ey_act = fk(t1_act,  t2_act)
    ee_err = math.hypot(ex_act - ex_cmd, ey_act - ey_cmd)

    print(f"     θ1  cmd={t1_goal:6.1f}°  act={t1_act:6.1f}°  err={abs(t1_act-t1_goal):.1f}°")
    print(f"     θ2  cmd={t2_goal:6.1f}°  act={t2_act:6.1f}°  err={abs(t2_act-t2_goal):.1f}°")
    print(f"     EE  cmd=({ex_cmd:.2f}, {ey_cmd:.2f})  "
          f"act=({ex_act:.2f}, {ey_act:.2f})  err={ee_err:.3f}\"")

    error_records.append({
        "name": wp_name, "color": wp_color,
        "t1_cmd": t1_goal, "t2_cmd": t2_goal,
        "t1_act": t1_act,  "t2_act": t2_act,
        "ee_cmd": (ex_cmd, ey_cmd),
        "ee_act": (ex_act, ey_act),
        "ee_error": ee_err,
    })

    print(f"     Holding {HOLD_SEC}s...")
    t_end = time.time() + HOLD_SEC
    while time.time() < t_end:
        t1a, t2a = read_actual(j1, j2)
        if monitor:
            monitor.update(t1_goal, t2_goal, t1a, t2a)
        time.sleep(0.05)


# ══════════════════════════════════════════════════════════════════════════════
# Dry-run: simulated motors
# ══════════════════════════════════════════════════════════════════════════════

class FakeMotor:
    def __init__(self): self._pos = 0.0; self.control_mode = None
    @property
    def position(self): return self._pos + np.random.normal(0, 0.4)
    @property
    def position_command(self): return self._pos
    @position_command.setter
    def position_command(self, v): self._pos = v


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def prompt_pair(label, default):
    dx, dy = default
    try:
        raw = input(f"  {label} x, y  (Enter = {dx}, {dy}): ").strip()
        if raw:
            x, y = [float(v) for v in raw.replace(",", " ").split()]
            return (x, y)
    except Exception:
        print(f"    Bad input — keeping ({dx}, {dy})")
    return default


def main():
    matplotlib.use("TkAgg")   # swap to "Qt5Agg" if needed
    plt.ion()

    # ── Runtime prompts ──────────────────────────────────────────────────────
    print("\n╔══════════════════════════════════╗")
    print("║   Lab 9  —  Arm Path Planner     ║")
    print("╚══════════════════════════════════╝")
    print("\nGoal coordinates (inches). Press Enter to keep default.")
    global A, B, C
    A = prompt_pair("A", A)
    B = prompt_pair("B", B)
    C = prompt_pair("C", C)
    WAYPOINTS[1] = A;  WAYPOINTS[2] = B;  WAYPOINTS[3] = C

    print("\nObstacles — press Enter to keep each one.")
    for k, obs in enumerate(OBSTACLES):
        x1, x2, y1, y2 = obs
        try:
            raw = input(f"  Obs {k+1}  x1 x2 y1 y2  (Enter = {x1} {x2} {y1} {y2}): ").strip()
            if raw:
                OBSTACLES[k] = [float(v) for v in raw.replace(",", " ").split()]
        except Exception:
            print(f"    Bad input — keeping Obs {k+1}")

    print(f"\nPlan: Start={START}  A={A}  B={B}  C={C}")
    print(f"Obs:  {OBSTACLES[0]}")
    print(f"      {OBSTACLES[1]}\n")

    # 1. Build C-space + plan
    cmap   = build_cspace(OBSTACLES, PAD)
    print("Planning paths...")
    path_A = downsample(plan_segment(START, A, cmap, "A"))
    path_B = downsample(plan_segment(A,     B, cmap, "B"))
    path_C = downsample(plan_segment(B,     C, cmap, "C"))
    print(f"  S→A: {len(path_A)} pts   A→B: {len(path_B)} pts   B→C: {len(path_C)} pts")

    named_cfgs = [
        (WAYPOINT_NAMES[0], WP_COLORS[0], *path_A[0]),
        (WAYPOINT_NAMES[1], WP_COLORS[1], *path_A[-1]),
        (WAYPOINT_NAMES[2], WP_COLORS[2], *path_B[-1]),
        (WAYPOINT_NAMES[3], WP_COLORS[3], *path_C[-1]),
    ]
    full_path = path_A + path_B[1:] + path_C[1:]

    # 2. Fig 1 — inspect plan before running
    show_plan(cmap, full_path, named_cfgs)
    input("\nFig 1 shown — inspect plan, then press Enter to start execution.\n")

    # 3. Connect motors
    if DRY_RUN:
        print("[DRY-RUN] Using simulated motors.")
        j1, j2 = FakeMotor(), FakeMotor()
    else:
        p = Plink(frequency=200, timeout=1.0)
        print("Connecting..."); p.connect(); print("Connected.")
        j1 = getattr(p, f"channel{CH_JOINT1}")
        j2 = getattr(p, f"channel{CH_JOINT2}")
        j1.control_mode = ControlMode.POSITION
        j2.control_mode = ControlMode.POSITION

    # 4. Fig 2 — live monitor
    monitor       = LiveMonitor()
    error_records = []

    t1_s, t2_s = path_A[0]
    print(f"\nMoving to start: θ1={t1_s:.1f}°  θ2={t2_s:.1f}°")
    move_to(j1, j2, t1_s, t2_s)
    wait_arrive(j1, j2, t1_s, t2_s, monitor, timeout=6.0)
    time.sleep(0.5)

    try:
        print("\n─── Demo run ───")
        execute_segment(j1, j2, path_A, "A", WP_COLORS[1], monitor, error_records)
        execute_segment(j1, j2, path_B, "B", WP_COLORS[2], monitor, error_records)
        execute_segment(j1, j2, path_C, "C", WP_COLORS[3], monitor, error_records)
        print("\n✓ Done.")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if not DRY_RUN:
            j1.position_command = j1.position
            j2.position_command = j2.position
        time.sleep(0.2)

    # 5. Fig 3 — error summary
    if error_records:
        show_error_summary(error_records)
    else:
        plt.ioff();  plt.show()


if __name__ == "__main__":
    main()
    