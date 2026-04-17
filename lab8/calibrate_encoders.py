import time
from motorgo import ControlMode, Plink

# ── Calibration settings ───────────────────────────────────────────────────────
MOTOR_POWER       = 0.5    # power to drive during calibration
NUM_RUNS          = 3      # number of runs to average


def main():
    known_distance = float(input("Enter the measured distance you will travel (in your units): "))

    plink = Plink()
    left_motor  = plink.channel1
    right_motor = plink.channel4
    plink.connect()

    left_motor.control_mode  = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    results = []

    for run in range(1, NUM_RUNS + 1):
        input(f"\nRun {run}/{NUM_RUNS} — Place robot at start mark, then press Enter to begin...")

        # Reset encoder reference
        start_enc = right_motor.position

        # Start motors
        left_motor.power_command  =  MOTOR_POWER
        right_motor.power_command = -MOTOR_POWER

        input("Press Enter to STOP the robot once it reaches the end mark...")

        # Stop motors and read encoder
        left_motor.power_command  = 0
        right_motor.power_command = 0
        end_enc   = right_motor.position
        raw_ticks = end_enc - start_enc
        if abs(raw_ticks) < 1e-6:
            print("  ERROR: Encoder did not move. Check encoder connection and retry.")
            continue
        dist_per_tick = known_distance / abs(raw_ticks)

        print(f"  Start enc : {start_enc:.4f}")
        print(f"  End enc   : {end_enc:.4f}")
        print(f"  Raw ticks : {raw_ticks:.4f}")
        print(f"  dist/tick : {dist_per_tick:.6f}")

        results.append(dist_per_tick)

    # ── Summary ────────────────────────────────────────────────────────────────
    avg = sum(results) / len(results)
    print("\n── Calibration Results ───────────────────────────────")
    for i, r in enumerate(results, 1):
        print(f"  Run {i}: {r:.6f}")
    print(f"  Average DIST_PER_TICK = {avg:.6f}")
    print("──────────────────────────────────────────────────────")
    print(f"\nPaste this into your main script:")
    print(f"  DIST_PER_TICK = {avg:.6f}")

    left_motor.power_command  = 0
    right_motor.power_command = 0


if __name__ == "__main__":
    main()