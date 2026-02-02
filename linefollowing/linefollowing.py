import time
import sys
import board
import adafruit_bh1750
from motorgo import Plink, ControlMode

LEFT_CH = 3
RIGHT_CH = 1

TARGET_LUX = 68.0
DT = 0.03

# Control in "normalized" space (0..1) then map to motor power that actually moves wheels
BASE_NORM = 0.25
KP = 0.020
POWER_CLIP_NORM = 0.60

# Measured motor deadband: wheels start moving ~0.45
MIN_MOVE = 0.40
MAX_MOVE = 0.50

STEER_SIGN = 1.0

def clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def apply_deadband(u, min_move, max_move):
    if abs(u) < 1e-6:
        return 0.0
    sign = 1.0 if u > 0 else -1.0
    mag = abs(u)
    mag = clip(mag, 0.0, 1.0)
    return sign * (min_move + (max_move - min_move) * mag)

def main():
    i2c = board.I2C()
    light = adafruit_bh1750.BH1750(i2c)

    p = Plink(frequency=200, timeout=1.0)
    left = None
    right = None

    try:
        print("Connecting...")
        p.connect()
        print("Connected.")

        left = getattr(p, f"channel{LEFT_CH}")
        right = getattr(p, f"channel{RIGHT_CH}")

        left.control_mode = ControlMode.POWER
        right.control_mode = ControlMode.POWER

        left.power_command = 0.0
        right.power_command = 0.0
        time.sleep(0.2)

        print("Line follow starting. Ctrl+C to stop.")
        time.sleep(0.2)

        while True:
            lux = float(light.lux)
            error = lux - TARGET_LUX

            steer = STEER_SIGN * KP * error

            left_u = BASE_NORM - steer
            right_u = BASE_NORM + steer

            left_u = clip(left_u, -POWER_CLIP_NORM, POWER_CLIP_NORM)
            right_u = clip(right_u, -POWER_CLIP_NORM, POWER_CLIP_NORM)

            left_cmd = apply_deadband(left_u, MIN_MOVE, MAX_MOVE)
            right_cmd = apply_deadband(right_u, MIN_MOVE, MAX_MOVE)

            left.power_command = left_cmd
            right.power_command = right_cmd

            print(f"lux={lux:6.1f} err={error:6.1f} uL={left_u: .3f} uR={right_u: .3f} L={left_cmd: .3f} R={right_cmd: .3f}")

            time.sleep(DT)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            if left is not None:
                left.power_command = 0.0
            if right is not None:
                right.power_command = 0.0
        except Exception:
            pass
        time.sleep(0.2)
        print("Stopped motors. Exiting.")
        sys.exit(0)

if __name__ == "__main__":
    main()
