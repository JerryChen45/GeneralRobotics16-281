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

LINE_LOST_THRESH = 35
UTURN_EXIT_THRESH = 15  # Exit U-turn when error is this small (line found!)

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
    in_uturn_mode = False  # Track if we're currently in U-turn

    print("=" * 50)
    print("LINE FOLLOWER - SMART U-TURN EXIT")
    print("=" * 50)
    print(f"LINE_LOST_THRESH = {LINE_LOST_THRESH} (enter U-turn)")
    print(f"UTURN_EXIT_THRESH = {UTURN_EXIT_THRESH} (exit U-turn)")
    print("Running... Press Ctrl+C to stop")
    print()

    try:
        while True:
            lux = float(light.lux)
            error = lux - TARGET_LUX

            # ---------- PD CONTROL ----------
            derivative = (error - prev_error) / DT
            prev_error = error
            steer = KP * error + KD * derivative

            # ========================================
            # SMART U-TURN STATE MACHINE
            # ========================================
            
            if not in_uturn_mode:
                # NORMAL TRACKING MODE
                if abs(error) > LINE_LOST_THRESH:
                    # Line lost! Enter U-turn mode
                    in_uturn_mode = True
                    print(f"LINE LOST! error={error:.1f} - Starting U-turn")
                
                # Normal PD tracking
                left_speed  = BASE_SPEED + steer
                right_speed = BASE_SPEED - steer
                
            else:
                # IN U-TURN MODE
                # Check if line is found (error is small)
                if abs(error) < UTURN_EXIT_THRESH:
                    # Line found! Exit U-turn immediately
                    in_uturn_mode = False
                    print(f"LINE FOUND! error={error:.1f} - Resuming tracking")
                    
                    # Resume normal tracking
                    left_speed  = BASE_SPEED + steer
                    right_speed = BASE_SPEED - steer
                else:
                    # Still lost, continue U-turn
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