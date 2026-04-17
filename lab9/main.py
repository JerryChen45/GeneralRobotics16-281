"""
main.py — drives the physical arm using paths planned by cspace2.py
Uses POWER control mode + encoder feedback (same approach as odom.py).
"""

import sys
import time
import math
import os

# Detect headless (no display) — use non-interactive Agg backend
HEADLESS = not os.environ.get("DISPLAY") and sys.platform != "win32"
import matplotlib
matplotlib.use("Agg" if HEADLESS else "TkAgg")
import matplotlib.pyplot as plt

# ── Import planning from cspace2 ──────────────────────────────────────────────
from cspace2 import (
    build_cspace, plan_segment, downsample,
    show_plan, show_error_summary, LiveMonitor,
    fk,
    START, A, B, C, WAYPOINT_NAMES, WP_COLORS,
    OBSTACLES, PAD, HOLD_SEC, STEP_SLEEP,
)

DRY_RUN = "--dry-run" in sys.argv
try:
    from motorgo import Plink, ControlMode
except ImportError:
    print("[WARN] motorgo not found — forcing --dry-run mode")
    DRY_RUN = True

# ── Calibration (from test_encoder / odom) ────────────────────────────────────
DEGREES_PER_TICK_J1 = 19      # channel 2 = joint 1
ENC_SIGN_J1         = 1

DEGREES_PER_TICK_J2 = 19      # channel 3 = joint 2
ENC_SIGN_J2         = -1

CH_JOINT1 = 2   # channel for θ1
CH_JOINT2 = 3   # channel for θ2

DIR_J1 =  1    # flip to -1 if motor spins wrong way
DIR_J2 = -1    # flipped

# ── PD gains ─────────────────────────────────────────────────────────────────
KP = 0.02   # proportional — increase if too slow, decrease if it oscillates
KD = 0.005  # derivative   — dampens overshoot

MAX_POWER     = 0.6   # clamp output to this
MIN_POWER     = 0.15  # minimum power to overcome friction
POS_TOLERANCE = 3.0   # degrees — stop when within this of target


# ── Encoder helpers ───────────────────────────────────────────────────────────
def enc_to_deg(raw, zero, enc_sign, deg_per_tick):
    return enc_sign * (raw - zero) * deg_per_tick

def read_actual(j1, j2, zero_j1, zero_j2):
    t1 = enc_to_deg(j1.position, zero_j1, ENC_SIGN_J1, DEGREES_PER_TICK_J1)
    t2 = enc_to_deg(j2.position, zero_j2, ENC_SIGN_J2, DEGREES_PER_TICK_J2)
    return t1, t2


# ── Power-mode move: drives both joints to (t1_tgt, t2_tgt) using PID ────────
def move_to_blocking(j1, j2, zero_j1, zero_j2,
                     t1_tgt, t2_tgt, monitor=None, timeout=10.0):
    # PD state for each joint
    prev_err1 = 0.0;  prev_err2 = 0.0
    prev_time = time.time()
    deadline  = time.time() + timeout

    while time.time() < deadline:
        now = time.time()
        dt  = now - prev_time
        prev_time = now
        if dt <= 0:
            dt = 0.001

        t1a, t2a = read_actual(j1, j2, zero_j1, zero_j2)

        err1 = t1_tgt - t1a
        err2 = t2_tgt - t2a

        d1 = (err1 - prev_err1) / dt
        d2 = (err2 - prev_err2) / dt
        prev_err1 = err1
        prev_err2 = err2

        out1 = KP * err1 + KD * d1
        out2 = KP * err2 + KD * d2

        # Clamp to [-MAX_POWER, MAX_POWER] and enforce minimum power
        out1 = max(-MAX_POWER, min(MAX_POWER, out1))
        out2 = max(-MAX_POWER, min(MAX_POWER, out2))
        if abs(out1) > 0: out1 = math.copysign(max(abs(out1), MIN_POWER), out1)
        if abs(out2) > 0: out2 = math.copysign(max(abs(out2), MIN_POWER), out2)

        done1 = abs(err1) <= POS_TOLERANCE
        done2 = abs(err2) <= POS_TOLERANCE

        if not DRY_RUN:
            j1.power_command = 0.0 if done1 else DIR_J1 * out1
            j2.power_command = 0.0 if done2 else DIR_J2 * out2

        if monitor:
            monitor.update(t1_tgt, t2_tgt, t1a, t2a)

        if done1 and done2:
            if not DRY_RUN:
                j1.power_command = 0.0
                j2.power_command = 0.0
            return True

        time.sleep(0.02)

    if not DRY_RUN:
        j1.power_command = 0.0
        j2.power_command = 0.0
    return False


# ── Execute one path segment ──────────────────────────────────────────────────
def execute_segment(j1, j2, zero_j1, zero_j2,
                    path, wp_name, wp_color, monitor, error_records):
    print(f"  → {wp_name} ({len(path)} waypoints)")
    for t1, t2 in path:
        move_to_blocking(j1, j2, zero_j1, zero_j2, t1, t2,
                         monitor=monitor, timeout=8.0)
        time.sleep(STEP_SLEEP)

    # Hold at goal
    t1_goal, t2_goal = path[-1]
    if monitor:
        monitor.mark_waypoint(wp_name, wp_color)

    print(f"     Holding {HOLD_SEC}s...")
    t_end = time.time() + HOLD_SEC
    while time.time() < t_end:
        t1a, t2a = read_actual(j1, j2, zero_j1, zero_j2)
        if monitor:
            monitor.update(t1_goal, t2_goal, t1a, t2a)
        time.sleep(0.05)

    t1_act, t2_act = read_actual(j1, j2, zero_j1, zero_j2)
    _, _, ex_cmd, ey_cmd = fk(t1_goal, t2_goal)
    _, _, ex_act, ey_act = fk(t1_act,  t2_act)
    ee_err = math.hypot(ex_act - ex_cmd, ey_act - ey_cmd)

    print(f"     θ1  cmd={t1_goal:6.1f}°  act={t1_act:6.1f}°  err={abs(t1_act-t1_goal):.1f}°")
    print(f"     θ2  cmd={t2_goal:6.1f}°  act={t2_act:6.1f}°  err={abs(t2_act-t2_goal):.1f}°")
    print(f"     EE  cmd=({ex_cmd:.2f},{ey_cmd:.2f})  act=({ex_act:.2f},{ey_act:.2f})  err={ee_err:.3f}\"")

    error_records.append({
        "name": wp_name, "color": wp_color,
        "t1_cmd": t1_goal, "t2_cmd": t2_goal,
        "t1_act": t1_act,  "t2_act": t2_act,
        "ee_cmd": (ex_cmd, ey_cmd),
        "ee_act": (ex_act, ey_act),
        "ee_error": ee_err,
    })


# ── Simulated motor for dry-run ───────────────────────────────────────────────
class FakeMotor:
    def __init__(self):
        self._pos = 0.0
        self.power_command = 0.0
        self.control_mode  = None
    @property
    def position(self):
        import numpy as np
        return self._pos + np.random.normal(0, 0.02)
    def nudge(self, target_deg, enc_sign):
        # Simulate motor drifting toward commanded angle
        target_raw = enc_sign * target_deg / DEGREES_PER_TICK_J1
        self._pos += (target_raw - self._pos) * 0.3


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    if not HEADLESS:
        plt.ion()

    print("\n╔══════════════════════════════════╗")
    print("║   Lab 9  —  Arm Path Planner     ║")
    print("╚══════════════════════════════════╝\n")

    # 1. Build C-space + plan paths
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

    # 2. Show/save plan
    fig1 = show_plan(cmap, full_path, named_cfgs)
    if HEADLESS:
        fig1.savefig("plan.png", dpi=120, bbox_inches="tight")
        print("Headless mode: plan saved to plan.png")
    input("\nPlan ready — press Enter to connect motors.\n")

    # 3. Connect motors
    if DRY_RUN:
        print("[DRY-RUN] Using simulated motors.")
        j1, j2 = FakeMotor(), FakeMotor()
        zero_j1 = zero_j2 = 0.0
    else:
        p = Plink()
        print("Connecting..."); p.connect(); print("Connected.")
        j1 = getattr(p, f"channel{CH_JOINT1}")
        j2 = getattr(p, f"channel{CH_JOINT2}")
        j1.control_mode = ControlMode.POWER
        j2.control_mode = ControlMode.POWER
        j1.power_command = 0.0
        j2.power_command = 0.0

        # Zero both joints
        input("\nPosition BOTH joints at 0°, then press ENTER to zero.")
        zero_j1 = j1.position
        zero_j2 = j2.position
        print(f"Zeroed: j1 encoder={zero_j1:.4f}  j2 encoder={zero_j2:.4f}\n")

    # 4. Live monitor + execute (disabled on headless — no display)
    monitor       = None if HEADLESS else LiveMonitor()
    error_records = []

    t1_s, t2_s = path_A[0]
    print(f"Moving to start: θ1={t1_s:.1f}°  θ2={t2_s:.1f}°")
    move_to_blocking(j1, j2, zero_j1, zero_j2, t1_s, t2_s, monitor, timeout=10.0)
    time.sleep(0.5)

    try:
        print("\n─── Demo run ───")
        execute_segment(j1, j2, zero_j1, zero_j2, path_A, "A", WP_COLORS[1], monitor, error_records)
        execute_segment(j1, j2, zero_j1, zero_j2, path_B, "B", WP_COLORS[2], monitor, error_records)
        execute_segment(j1, j2, zero_j1, zero_j2, path_C, "C", WP_COLORS[3], monitor, error_records)
        print("\nDone.")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if not DRY_RUN:
            j1.power_command = 0.0
            j2.power_command = 0.0
        time.sleep(0.2)

    # 5. Error summary
    if error_records:
        fig3 = show_error_summary(error_records)
        if HEADLESS and fig3:
            fig3.savefig("error_summary.png", dpi=120, bbox_inches="tight")
            print("Headless mode: error summary saved to error_summary.png")
    elif not HEADLESS:
        plt.ioff(); plt.show()


if __name__ == "__main__":
    main()
