import math
import time
import random
import board
import numpy as np
import adafruit_bh1750
import adafruit_vl53l4cx
from motorgo import Plink, ControlMode


# -----------------------------------------------------------------
#  TUNABLE CONSTANTS
# -----------------------------------------------------------------

NUM_SECTORS      = 16
SECTOR_ANGLE_RAD = 2 * math.pi / NUM_SECTORS

BLOCK_THRESHOLD_CM = 35.0
P_HIT              = 0.75
P_MISS             = 1 - P_HIT

NUM_PARTICLES    = 3200
MOTION_NOISE_STD = 0.3

CONFIDENCE_THRESHOLD = 0.6
TIME_LIMIT_S         = 65

WHEEL_RADIUS   = 1.125
WHEELBASE      = 6.25
TICKS_PER_REV  = 6.4845
LEFT_ENC_SIGN  = -1
RIGHT_ENC_SIGN = 1
DT             = 0.01

LEFT_CH    = 1
RIGHT_CH   = 4
RIGHT_DIR  = -1
LEFT_DIR   = 1
BASE_POWER = 0.5
TARGET_LUX = 108.5
KP         = 0.033
KD         = 0.0063
FOLLOW_DT  = 0.1
SECTOR_DIST = 5.5

# -----------------------------------------------------------------
#  HARDWARE
# -----------------------------------------------------------------

def init_hardware():
    i2c   = board.I2C()
    light = adafruit_bh1750.BH1750(i2c)
    tof   = adafruit_vl53l4cx.VL53L4CX(i2c)
    tof.start_ranging()

    p = Plink(frequency=200, timeout=1.0)
    p.connect()
    print("Plink connected.")

    left  = getattr(p, f"channel{LEFT_CH}")
    right = getattr(p, f"channel{RIGHT_CH}")
    left.control_mode  = ControlMode.POWER
    right.control_mode = ControlMode.POWER
    left.power_command  = 0.0
    right.power_command = 0.0
    time.sleep(0.2)

    return p, left, right, light, tof


def shutdown_motors(left, right):
    left.power_command  = 0.0
    right.power_command = 0.0


# -----------------------------------------------------------------
#  ODOMETRY
# -----------------------------------------------------------------

def drive_model(s, v, omega):
    return np.array([
        v * math.cos(s[2]),
        v * math.sin(s[2]),
        omega
    ])


def rk4_step(state, v, omega, dt):
    k1 = drive_model(state, v, omega)
    k2 = drive_model(state + 0.5 * dt * k1, v, omega)
    k3 = drive_model(state + 0.5 * dt * k2, v, omega)
    k4 = drive_model(state + dt * k3, v, omega)
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


def update_odometry(state, prev_left, prev_right, accumulated_dist, left, right):
    curr_left  = left.position
    curr_right = right.position

    delta_left  = LEFT_ENC_SIGN  * (curr_left  - prev_left)
    delta_right = RIGHT_ENC_SIGN * (curr_right - prev_right)

    dist_left  = (delta_left  / TICKS_PER_REV) * (2 * math.pi) * WHEEL_RADIUS
    dist_right = (delta_right / TICKS_PER_REV) * (2 * math.pi) * WHEEL_RADIUS

    v     = 0.5 * (dist_left + dist_right) / DT
    omega = (dist_right - dist_left) / (WHEELBASE * DT)

    state = rk4_step(state, v, omega, DT)

    avg_dist = 0.5 * (dist_left + dist_right)
    accumulated_dist += avg_dist

    return state, curr_left, curr_right, accumulated_dist


# -----------------------------------------------------------------
#  LINE FOLLOWER
# -----------------------------------------------------------------

def line_follow_step(left, right, light, prev_error):
    lux    = float(light.lux)
    error  = lux - TARGET_LUX
    capped = max(-10.0, min(10.0, error))

    steer = KP * capped + KD * (capped - prev_error) / FOLLOW_DT

    left_pwr  = max(-1.0, min(1.0, BASE_POWER - steer))
    right_pwr = max(-1.0, min(1.0, BASE_POWER + steer))

    left.power_command  = LEFT_DIR  * left_pwr
    right.power_command = RIGHT_DIR * right_pwr

    return lux, capped


# -----------------------------------------------------------------
#  ToF SENSOR
# -----------------------------------------------------------------

def read_tof_cm(tof):
    try:
        tof.clear_interrupt()
        reading = tof.distance
        if reading is None:
            return None
        return float(reading)
    except Exception as e:
        print(f"[WARN] ToF error: {e}")
        return None


# -----------------------------------------------------------------
#  PARTICLE FILTER
# -----------------------------------------------------------------

def init_particles():
    return [random.uniform(0, NUM_SECTORS) for _ in range(NUM_PARTICLES)]


def motion_update(particles):
    for i in range(NUM_PARTICLES):
        noise = random.gauss(0, MOTION_NOISE_STD)
        particles[i] = (particles[i] + 1 + noise) % NUM_SECTORS
    return particles


def sensor_update(particles, z, map_bits):
    weights = []
    for p in particles:
        sector = int(p) % NUM_SECTORS
        if map_bits[sector] == z:
            weights.append(P_HIT)
        else:
            weights.append(P_MISS)

    total = sum(weights)
    weights = [w / total for w in weights]

    new_particles = []
    r = random.uniform(0, 1.0 / NUM_PARTICLES)
    c = weights[0]
    j = 0
    for i in range(NUM_PARTICLES):
        threshold = r + i * (1.0 / NUM_PARTICLES)
        while c < threshold:
            j += 1
            c += weights[j]
        new_particles.append(particles[j])

    return new_particles


def best_estimate(particles):
    sector_counts = [0] * NUM_SECTORS
    for p in particles:
        sector_counts[int(p) % NUM_SECTORS] += 1

    best = max(range(NUM_SECTORS), key=lambda x: sector_counts[x])
    prob = sector_counts[best] / NUM_PARTICLES
    return best, prob


def print_status(particles, label=""):
    sector, prob = best_estimate(particles)
    tag = f" {label}" if label else ""
    print(f"[PF{tag}]  best={sector:2d}  confidence={prob:.1%}")


# -----------------------------------------------------------------
#  DRIVE N SECTORS
# -----------------------------------------------------------------

def drive_n_sectors(left, right, light, n):
    prev_error = 0.0
    state = np.array([0.0, 0.0, 0.0])
    prev_l = left.position
    prev_r = right.position
    acc_dist = 0.0
    sectors_passed = 0

    while sectors_passed < n:
        _, prev_error = line_follow_step(left, right, light, prev_error)
        state, prev_l, prev_r, acc_dist = update_odometry(
            state, prev_l, prev_r, acc_dist, left, right
        )
        time.sleep(FOLLOW_DT)

        if acc_dist >= SECTOR_DIST:
            acc_dist -= SECTOR_DIST
            sectors_passed += 1
            print(f"[NAV] passed sector {sectors_passed}/{n}")

    left.power_command = 0.0
    right.power_command = 0.0


# -----------------------------------------------------------------
#  MAIN
# -----------------------------------------------------------------

def localize_and_navigate(map_bits, goal_sector, sector_count = 0):
    p, left, right, light, tof = init_hardware()

    particles  = init_particles()
    prev_error = 0.0
    state      = np.array([0.0, 0.0, 0.0])
    prev_l     = left.position
    prev_r     = right.position
    acc_dist  = 0.0

    print("=== Lab 6 Localization ===")
    print(f"Map : {map_bits}")
    print(f"Goal: sector {goal_sector}")
    print(f"Time limit: {TIME_LIMIT_S}s\n")

    start = time.time()

    try:
        while True:
            elapsed = time.time() - start

            if elapsed >= TIME_LIMIT_S:
                sector, prob = best_estimate(particles)
                print(f"[TIME] {elapsed:.1f}s — forcing: sector {sector} ({prob:.1%})")
                remaining = (goal_sector - sector) % NUM_SECTORS
                drive_n_sectors(left, right, light, remaining)
                return

            _, prev_error = line_follow_step(left, right, light, prev_error)
            state, prev_l, prev_r, acc_dist = update_odometry(
                state, prev_l, prev_r, acc_dist, left, right
            )
            time.sleep(FOLLOW_DT)

            if acc_dist < SECTOR_DIST:
                continue

            acc_dist -= SECTOR_DIST
            sector_count += 1
            print(f"\n--- Sector crossing #{sector_count} (t={elapsed:.1f}s) ---")

            particles = motion_update(particles)

            tof_dist = read_tof_cm(tof)
            if tof_dist is None:
                print_status(particles, "(no update)")
                continue

            z = 1 if tof_dist < BLOCK_THRESHOLD_CM else 0
            particles = sensor_update(particles, z, map_bits)
            print_status(particles, f"z={z}  tof={tof_dist:.1f}cm")

            sector, prob = best_estimate(particles)
            if prob >= CONFIDENCE_THRESHOLD:
                print(f"\n[CONVERGED] sector={sector}  conf={prob:.1%}  t={elapsed:.1f}s")
                remaining = (goal_sector - sector) % NUM_SECTORS
                print(f"[NAV] {remaining} sector(s) to goal {goal_sector}")
                drive_n_sectors(left, right, light, remaining)
                return

    finally:
        shutdown_motors(left, right)
        print("[DONE] Motors stopped.")


if __name__ == "__main__":
    MAP  = [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0]
    GOAL = 5
    localize_and_navigate(MAP, GOAL)