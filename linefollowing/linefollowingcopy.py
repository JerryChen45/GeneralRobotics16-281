import time
import sys
import board
import adafruit_bh1750
from motorgo import Plink, ControlMode

# ================= CONFIG =================
LEFT_CH = 1
RIGHT_CH = 3

TARGET_LUX = 70
DT = 0.005

BASE_SPEED = 0.30
MAX_SPEED  = 0.8

KP = 0.175
KD = 0.225

LEFT_DIR  = -1.0
RIGHT_DIR =  1.0

LINE_LOST_THRESH = 15   # lux error for U-turn
STEER_FRACTION   = 0.9    # max steer relative to base speed
# =========================================


def clip(x, lo, hi):
    return max(lo, min(x, hi))


def main():
    # ---- Sensors ----
    i2c = board.I2C()
    light = adafruit_bh1750.BH1750(i2c)

    # ---- Motors ----
    p = Plink(frequency=200, timeout=1.0)
    p.connect()

    left  = getattr(p, f"channel{LEFT_CH}")
    right = getattr(p, f"channel{RIGHT_CH}")

    left.control_mode  = ControlMode.POWER
    right.control_mode = ControlMode.POWER

    left.power_command  = 0.0
    right.power_command = 0.0
    time.sleep(0.3)

    prev_error = 0.0

    try:
        while True:
            lux = float(light.lux)

            # ---------- PD CONTROL ----------
            error = lux - TARGET_LUX
            derivative = (error - prev_error) / DT
            prev_error = error

            steer = KP * error + KD * derivative
            
            # ---------- NORMAL TRACKING ----------
            left_speed  = (BASE_SPEED + steer)
            right_speed = (BASE_SPEED - steer)

            # ---------- U-TURN MODE ----------
            if abs(error) > LINE_LOST_THRESH:
                # Pivot hard toward the line
                turn = clip(0.6 * error / abs(error), -1.0, 1.0)
                left_speed  = -turn
                right_speed =  turn

            # ---------- FINAL CLIP ----------
            left_speed  = clip(left_speed,  -MAX_SPEED, MAX_SPEED)
            right_speed = clip(right_speed, -MAX_SPEED, MAX_SPEED)

            # ---------- APPLY MOTOR DIR ----------
            left.power_command  = LEFT_DIR  * left_speed
            right.power_command = RIGHT_DIR * right_speed

            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        left.power_command  = 0.0
        right.power_command = 0.0
        time.sleep(0.2)
        sys.exit(0)


if __name__ == "__main__":
    main()
