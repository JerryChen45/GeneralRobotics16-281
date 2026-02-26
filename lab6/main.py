import math
import time
import board
import adafruit_bh1750
import adafruit_vl53l4cx
from motorgo import Plink, ControlMode


# -----------------------------------------------------------------
#  TUNABLE CONSTANTS
# -----------------------------------------------------------------

# Map & course
NUM_SECTORS       = 16
SECTOR_ANGLE_RAD  = 2 * math.pi / NUM_SECTORS   # 22.5 deg = 0.3927 rad

# Sensor model
# Threshold is wide (45 cm) to handle blocks shifted up to 6 inches (~15 cm)
# by TAs during standard and challenge rounds.
# P_HIT is lowered to 0.75 so a single bad reading (rotated/moved block)
# does not collapse belief — the filter recovers over subsequent updates.
BLOCK_THRESHOLD_CM = 45.0
P_HIT              = 0.75       # P(correct reading | true state)
P_MISS             = 1 - P_HIT

# Motion model: P(advancing 0, 1, or 2 sectors | commanded 1 sector)
P_MOVE = {0: 0.15, 1: 0.70, 2: 0.15}

# Convergence
CONFIDENCE_THRESHOLD = 0.75
TIME_LIMIT_S         = 75.0    # bail-out a few seconds before hard limit

# Odometry (from Odometry.py)
WHEEL_RADIUS    = 1.125        # inches
WHEELBASE       = 6.25         # inches
TICKS_PER_REV   = 6.4845
LEFT_ENC_SIGN   = -1
RIGHT_ENC_SIGN  = 1

# Line follower (from linefollowing.py)
LEFT_CH         = 1
RIGHT_CH        = 4
RIGHT_DIR       = -1           # right motor is physically reversed
LEFT_DIR        = 1
BASE_POWER      = 0.5
TARGET_LUX      = 85.0
KP              = 0.08
KD              = 0.009
FOLLOW_DT       = 0.1          # seconds between line-follower iterations


# -----------------------------------------------------------------
#  HARDWARE INIT
# -----------------------------------------------------------------

def init_hardware():
    """Initialise all hardware. Returns (plink, left_ch, right_ch, light, tof)."""
    i2c   = board.I2C()
    light = adafruit_bh1750.BH1750(i2c)

    dist = adafruit_vl53l4cx.VL53L4CX(i2c)
    dist.start_ranging()

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

    return p, left, right, light, dist


def shutdown_motors(left, right):
    left.power_command  = 0.0
    right.power_command = 0.0


# -----------------------------------------------------------------
#  ODOMETRY  (angle accumulator — bug fixed from original Odometry.py)
# -----------------------------------------------------------------

class Odometry:
    """
    Tracks accumulated heading change (radians) from encoder deltas.
    Only heading is needed for sector detection.

    Bug fix vs. original Odometry.py: accumulated_angle is now an instance
    variable, not an undefined local that caused NameError at runtime.
    """

    def __init__(self, left_ch, right_ch):
        self._left  = left_ch
        self._right = right_ch
        self.reset()

    def reset(self):
        """Zero the heading accumulator and store current encoder positions."""
        self._prev_left        = self._left.position
        self._prev_right       = self._right.position
        self.accumulated_angle = 0.0   # radians

    def update(self) -> float:
        """
        Read encoders, compute delta heading, accumulate.
        Returns accumulated_angle (radians) since last reset().
        """
        curr_left  = self._left.position
        curr_right = self._right.position

        delta_left  = LEFT_ENC_SIGN  * (curr_left  - self._prev_left)
        delta_right = RIGHT_ENC_SIGN * (curr_right - self._prev_right)

        self._prev_left  = curr_left
        self._prev_right = curr_right

        dist_left  = (delta_left  / TICKS_PER_REV) * (2 * math.pi) * WHEEL_RADIUS
        dist_right = (delta_right / TICKS_PER_REV) * (2 * math.pi) * WHEEL_RADIUS

        delta_theta = (dist_right - dist_left) / WHEELBASE
        self.accumulated_angle += delta_theta

        return self.accumulated_angle


# -----------------------------------------------------------------
#  LINE FOLLOWER  (PD controller, from linefollowing.py)
# -----------------------------------------------------------------

class LineFollower:
    def __init__(self, left_ch, right_ch, light_sensor):
        self._left  = left_ch
        self._right = right_ch
        self._light = light_sensor
        self._prev_error = 0.0

    def step(self) -> float:
        """One PD iteration. Returns current lux reading."""
        lux    = float(self._light.lux)
        error  = lux - TARGET_LUX
        capped = max(-10.0, min(10.0, error))

        steer = KP * capped + KD * (capped - self._prev_error) / FOLLOW_DT
        self._prev_error = capped

        left_pwr  = max(-1.0, min(1.0, BASE_POWER + steer))
        right_pwr = max(-1.0, min(1.0, BASE_POWER - steer))

        self._left.power_command  = LEFT_DIR  * left_pwr
        self._right.power_command = RIGHT_DIR * right_pwr

        return lux


# -----------------------------------------------------------------
#  ToF SENSOR READ
# -----------------------------------------------------------------

def read_tof_cm(dist) -> float:
    """
    Read distance in cm from the VL53L4CX.
    Mirrors the pattern from the sensor test script:
      dist.clear_interrupt() then dist.distance (already in cm).
    Returns None if the sensor has no data ready yet.
    """
    dist.clear_interrupt()
    reading = dist.distance      # VL53L4CX returns cm directly
    if reading is None:
        raise ValueError("VL53L4CX returned None — no data ready")
    return float(reading)


# -----------------------------------------------------------------
#  DISCRETE BAYES FILTER
# -----------------------------------------------------------------

class BayesFilter:
    """
    Discrete Bayes filter over NUM_SECTORS states.
    Motion update (prediction) + sensor update (correction).
    """

    def __init__(self, map_bits: list):
        assert len(map_bits) == NUM_SECTORS
        self.map    = map_bits
        self.belief = [1.0 / NUM_SECTORS] * NUM_SECTORS

    def motion_update(self):
        """Convolve belief with P_MOVE kernel (robot moved ~1 sector CW)."""
        new_belief = [0.0] * NUM_SECTORS
        for x_prev in range(NUM_SECTORS):
            for delta, prob in P_MOVE.items():
                x_next = (x_prev + delta) % NUM_SECTORS
                new_belief[x_next] += prob * self.belief[x_prev]
        self.belief = new_belief

    def sensor_update(self, z: int):
        """
        z : 1 = block detected, 0 = no block.
        Bel(x) = eta * P(z|x) * Bel_pred(x), then normalise.
        """
        for x in range(NUM_SECTORS):
            p_obs = P_HIT if self.map[x] == z else P_MISS
            self.belief[x] *= p_obs

        total = sum(self.belief)
        if total > 0:
            self.belief = [b / total for b in self.belief]
        else:
            # Numerical underflow: reset to uniform
            self.belief = [1.0 / NUM_SECTORS] * NUM_SECTORS

    def best_estimate(self):
        """Return (most_likely_sector, probability)."""
        best = max(range(NUM_SECTORS), key=lambda x: self.belief[x])
        return best, self.belief[best]

    def is_converged(self) -> bool:
        _, prob = self.best_estimate()
        return prob >= CONFIDENCE_THRESHOLD

    def print_status(self, label: str = ""):
        sector, prob = self.best_estimate()
        tag = f" {label}" if label else ""
        print(f"[Bayes{tag}]  best={sector:2d}  confidence={prob:.1%}")


# -----------------------------------------------------------------
#  DRIVE N SECTORS (angle-based, used after convergence)
# -----------------------------------------------------------------

def drive_n_sectors(follower: LineFollower, odo: Odometry, n: int):
    """Drive exactly n sectors by heading, continuing to follow the line."""
    for _ in range(n):
        odo.reset()
        while odo.update() < SECTOR_ANGLE_RAD:
            follower.step()
            time.sleep(FOLLOW_DT)


# -----------------------------------------------------------------
#  MAIN LOCALIZATION LOOP
# -----------------------------------------------------------------

def localize_and_navigate(map_bits: list, goal_sector: int):
    """
    Full pipeline:
      1. Line-follow and accumulate heading.
      2. Every SECTOR_ANGLE_RAD of heading -> motion update + sensor update.
      3. Once belief converges, drive remaining sectors to GOAL.
    """
    p, left, right, light, tof = init_hardware()
    follower = LineFollower(left, right, light)
    odo      = Odometry(left, right)
    bayes    = BayesFilter(map_bits)

    print("=== Lab 6 Localization ===")
    print(f"Map : {map_bits}")
    print(f"Goal: sector {goal_sector}")
    print(f"Time limit: {TIME_LIMIT_S}s\n")

    start = time.time()
    odo.reset()

    try:
        while True:
            elapsed = time.time() - start

            # Time-limit bail-out
            if elapsed >= TIME_LIMIT_S:
                sector, prob = bayes.best_estimate()
                print(f"[TIME] {elapsed:.1f}s — forcing decision: "
                      f"sector {sector} ({prob:.1%})")
                _navigate_to_goal(follower, odo, sector, goal_sector)
                return

            # Line-follow until one sector angle has elapsed
            odo.reset()
            while True:
                angle = odo.update()
                follower.step()
                time.sleep(FOLLOW_DT)

                if angle >= SECTOR_ANGLE_RAD:
                    # Carry overshoot into next sector window
                    odo.accumulated_angle -= SECTOR_ANGLE_RAD
                    break

                if time.time() - start >= TIME_LIMIT_S:
                    break

            # Motion update
            bayes.motion_update()

            # Sensor update
            try:
                tof_dist = read_tof_cm(tof)
                z = 1 if tof_dist < BLOCK_THRESHOLD_CM else 0
            except Exception as e:
                print(f"[WARN] ToF error: {e} — skipping sensor update")
                bayes.print_status("(no update)")
                continue

            bayes.sensor_update(z)
            bayes.print_status(f"z={z}  tof={tof_dist:.1f}cm")

            # Convergence check
            if bayes.is_converged():
                sector, prob = bayes.best_estimate()
                print(f"\n[CONVERGED] sector={sector}  conf={prob:.1%}  "
                      f"t={elapsed:.1f}s")
                _navigate_to_goal(follower, odo, sector, goal_sector)
                return

    finally:
        # Always kill motors, even on exception
        shutdown_motors(left, right)
        print("[DONE] Motors stopped.")


def _navigate_to_goal(follower: LineFollower, odo: Odometry,
                      current_sector: int, goal_sector: int):
    remaining = (goal_sector - current_sector) % NUM_SECTORS
    print(f"[NAV] {remaining} sector(s) to goal {goal_sector}")
    drive_n_sectors(follower, odo, remaining)
    print(f"[DONE] Reached goal sector {goal_sector}.")


# -----------------------------------------------------------------
#  ENTRY POINT
# -----------------------------------------------------------------

if __name__ == "__main__":
    # Set these before each trial
    MAP  = [1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]   # example from writeup
    GOAL = 5                                                      # change per trial

    localize_and_navigate(MAP, GOAL)
