import time
import sys
from motorgo import Plink, ControlMode

LEFT_CH = 1
RIGHT_CH = 4

def clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def main():
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

        # Ramp to overcome static friction
        print("Ramping 0.00 -> 0.40 (watch wheels). Ctrl+C to stop.")
        pwr = 0.0
        while pwr <= 1:
            left.power_command = pwr
            right.power_command = -pwr
            print(f"pwr={pwr:.2f}")
            time.sleep(0.6)
            pwr += 0.05

    except KeyboardInterrupt:
        pass

    finally:
        # Always stop motors
        try:
            if left is not None:
                left.power_command = 0.0
            if right is not None:
                right.power_command = 0.0
        except Exception:
            pass

        # Give hardware a moment to settle
        time.sleep(0.2)

        print("Stopped motors. Exiting cleanly.")
        sys.exit(0)

if __name__ == "__main__":
    main()
