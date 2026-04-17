import math
import time

from motorgo import ControlMode, Plink

# ── Robot constants ────────────────────────────────────────────────────────────
DEGREES_PER_TICK = 19
ENC_SIGN         = 1        # from test_encoder calibration
MOTOR_POWER      = 0.5      # drive power [0, 1] — keep low for precision
TOLERANCE_DEG    = 1.0      # stop when within this many degrees of target
DT               = 0.01     # seconds between control loop updates


def enc_to_deg(raw, zero):
    """Convert raw encoder position to degrees from zero."""
    return ENC_SIGN * (raw - zero) * DEGREES_PER_TICK


# ── Move link 1 by a given number of degrees ──────────────────────────────────
def move_by_degrees(motor, target_degrees, zero=0.0):
    """
    Rotate link 1 by target_degrees relative to the zeroed position.
    Positive = one direction, negative = the other.
    """
    start_deg    = enc_to_deg(motor.position, zero)
    goal_deg     = start_deg + target_degrees

    ticks_needed = target_degrees / DEGREES_PER_TICK
    target_enc   = motor.position + (ENC_SIGN * ticks_needed)
    direction    = 1 if target_enc > motor.position else -1

    print(f"Moving by {target_degrees:.2f}°  (from {start_deg:.2f}° → {goal_deg:.2f}°)")

    while True:
        curr_deg    = enc_to_deg(motor.position, zero)
        deg_left    = goal_deg - curr_deg

        print(f"  current: {curr_deg:7.2f}°   left: {deg_left:7.2f}°", end="\r")

        if abs(deg_left) <= TOLERANCE_DEG:
            motor.power_command = 0
            print(f"\nDone! Stopped at {enc_to_deg(motor.position, zero):.2f}°")
            break

        motor.power_command = direction * MOTOR_POWER
        time.sleep(DT)


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    plink = Plink()

    link1_motor = plink.channel3   # change to whichever channel link 1 is on

    plink.connect()

    link1_motor.control_mode = ControlMode.POWER

    input("Position the link at 0 degrees, then press ENTER to zero.")
    zero = link1_motor.position
    print(f"Zeroed at encoder={zero:.4f}\n")

    try:
        while True:
            user_input = input("\nEnter degrees to turn link 1 (or 'q' to quit): ")
            if user_input.strip().lower() == 'q':
                break
            try:
                degrees = float(user_input)
                move_by_degrees(link1_motor, degrees, zero)
            except ValueError:
                print("Please enter a number.")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        link1_motor.power_command = 0
        print("Motor stopped.")


if __name__ == "__main__":
    main()