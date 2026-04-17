# Lab 9 — Two-Link RR Robot Arm Path Planner

16-281 General Robotics | Spring 2026

---

## Overview

This project implements inverse kinematics and path planning for a two-link RR robot arm navigating a known 2D workspace with obstacles. The arm must travel from a fixed start position through three waypoints A → B → C, stopping for 3 seconds at each so the grader can record the end-effector position.

The code is split into two files intentionally:

- **`cspace2.py`** — all planning logic (kinematics, C-space construction, A\*). No motor code. Can be imported and tested independently on any machine.
- **`main.py`** — motor execution only. Imports everything from `cspace2.py`, connects to the Plink, and drives the arm using a PD loop with encoder feedback.

This separation means the C-space build and path planning run on your laptop while you're designing the trajectory, and the same planned paths are executed on hardware without recomputing anything.

---

## Arm Specification

| Parameter | Value |
|-----------|-------|
| Link 1 (L1) | 4.0 inches |
| Link 2 (L2) | 4.5 inches |
| θ1 range | 0° to 180° |
| θ2 range | -180° to 180° |
| Orientation | Horizontal (no gravity compensation) |

---

## How to Run

```bash
# Dry run — planning and visualization only, no motors
python main.py --dry-run

# Full run — connects to Plink and drives the arm
python main.py
```

On demo day, update A, B, C when prompted at startup. Press Enter to keep defaults.

---

## Algorithm

### 1. Inverse Kinematics

Given a target (x, y), IK solves for the joint angles using the law of cosines:

```
cos(θ2) = (x² + y² - L1² - L2²) / (2·L1·L2)

θ1 = atan2(y, x) - atan2(L2·sin(θ2), L1 + L2·cos(θ2))
```

This gives up to two solutions — elbow-up and elbow-down. Both are passed to A\* and the shortest collision-free path wins.

### 2. Configuration Space

The C-space is a 2D grid indexed by (θ1, θ2) at 2° resolution, giving a 91×181 grid. Every cell is classified as:

| Value | Meaning |
|-------|---------|
| 0 | Free |
| 1 | Hard collision — arm link intersects raw obstacle |
| 2 | Obstacle pad zone — within 0.2" of obstacle boundary |
| 3 | Wall pad zone — within 0.1" of arena edge |

The grid is built using vectorised numpy — all 16,471 configurations are checked simultaneously using broadcasted arrays of arm sample points. This runs in ~0.08 seconds rather than 2+ seconds for an equivalent Python loop.

For each configuration, 30 points are sampled along each link segment and checked against every obstacle rectangle. Obstacle coordinates are stored raw (unpadded), and the pad zones are computed separately so the planning layer can distinguish hard from soft boundaries.

### 3. A\* Path Planning

A\* searches the C-space grid from the start cell to the goal cell. Key design decisions:

**Heuristic** — Euclidean distance in degree-space, with θ2 wrap-around:
```
h = sqrt((Δθ1)² + (min(|Δθ2|, 360-|Δθ2|))²)
```
This is admissible because you can never travel faster than a straight line in degree-space, so the heuristic never overestimates.

**Neighbour cost** — 8-connected grid (including diagonals). Each step costs the Euclidean degree-distance traveled. Padded cells add a penalty (40 for obstacle pad, 20 for wall pad) rather than hard-blocking, so A\* routes around them when possible but can still pass through them if a waypoint is very close to an obstacle.

**Stale entry check** — the heap stores `(f, g, i, j)` separately. When a cell is re-settled at lower cost, a new entry is pushed and the old one is discarded when popped via `if g > dist[cell]: continue`. This is critical — an earlier bug compared `f` against `dist` instead of `g`, which caused A\* to skip almost every node it popped and return no path.

**IK disambiguation** — for each segment (S→A, A→B, B→C), A\* is run for every combination of start and goal IK solutions. The shortest path across all combinations is selected.

### 4. Motor Execution

Motors run in **POWER mode** with a PD feedback loop using encoder readings:

```
error     = target_angle - actual_angle
output    = Kp * error + Kd * (d_error/dt)
power     = clamp(output, MIN_POWER, MAX_POWER)
```

The arm steps through each waypoint in the downsampled path, calling `move_to_blocking()` at each one. At the final waypoint of each segment, the arm holds position for 3 seconds before moving on.

**Position tolerance vs EE error** — the PD loop stops when both joints are within `POS_TOLERANCE` degrees of target. The worst-case EE error from joint error T (degrees) is bounded by:

```
|ΔEE| ≤ (L1 + 2·L2) · T_radians = 13 · T_radians
```

At 3° tolerance this gives ~0.68" worst case, which can miss the 0.5" full-point threshold. The tolerance is set to 2° to guarantee ≤ 0.45" worst-case EE error.

---

## Challenges

### A\* stale entry bug

The hardest bug to find. A\* returned `None` for every segment despite all waypoints being in the same connected component of free C-space. The issue was in the stale-entry check:

```python
# WRONG — compares f (= g + h) against dist (= g)
if f > dist[ci, cj] + 1e-4: continue   # h > 0 always makes this true

# CORRECT — compare g against dist
if g > dist[ci, cj] + 1e-4: continue
```

Because `f = g + h` and `h > 0` for almost every non-goal cell, every node popped from the heap was immediately discarded. The heap would drain completely without ever reaching the goal. The fix was storing `g` as a separate field in the heap tuple.

### Wall padding flagging every configuration

The initial wall-pad implementation checked all arm sample points — including the fixed base at `(0, 0)` — for proximity to the arena boundary. Since the base sits exactly on the arena floor (`y = 0`), every single configuration was flagged as wall-pad, leaving `free = 0` cells. The fix was to exclude the base point (`t = 0`) when building the wall-pad sample array, so only the moving parts of the arm trigger the boundary check.

### Obstacle coordinate padding

The lab-provided obstacle coordinates already included physical padding from measurement. Applying an additional `PAD = 0.2` in code created double-padding that pushed goal B (at `y = 4`) into a padded zone because obstacle 2 started at `y = 4.8`, and `4.8 - 0.2 = 4.6` — but combined with the already-padded raw coordinates it effectively blocked configurations near B. Keeping the raw obstacle coordinates unpadded and computing pad zones separately in the C-space builder resolved this.

### Encoder calibration

`DEGREES_PER_TICK` must be measured physically, not assumed. A wrong value introduces a systematic angle offset that scales with distance from zero — small errors at the start position become large EE errors at A, B, C. The calibration procedure is to command a known angle (90°), measure the physical result with a protractor, and adjust the constant.

---

## Scoring Reference

| EE error | Points |
|----------|--------|
| ≤ 0.5"   | Full (10/20/30 for A/B/C) |
| ≤ 1.0"   | Half |
| > 1.0"   | 0 |

Penalties: -5 pts/second over 35s, -15 pts for any obstacle or workspace boundary collision.
