import time

from motorgo import ControlMode, Plink

# ── Setup ──────────────────────────────────────────────────────────────────────
plink = Plink()

link1_motor = plink.channel3   # change to your channel

plink.connect()

link1_motor.control_mode = ControlMode.POWER


# ── Calibration ────────────────────────────────────────────────────────────────
def calibrate():
    print("=== ENCODER CALIBRATION ===")
    print("This will calculate DEGREES_PER_TICK for your arm.\n")

    # Step 1: zero the encoder at starting position
    input("Position link 1 at a known angle (e.g. 0 degrees). Press ENTER when ready.")
    start_enc = link1_motor.position
    print(f"  Start encoder: {start_enc:.4f}")

    # Step 2: move the arm to another known angle
    known_degrees = float(input("\nEnter the angle you will manually move the arm TO (e.g. 90): "))
    input(f"Now manually move the arm to {known_degrees} degrees. Press ENTER when done.")

    end_enc = link1_motor.position
    print(f"  End encoder: {end_enc:.4f}")

    # Step 3: calculate
    ticks = end_enc - start_enc
    if ticks == 0:
        print("\nERROR: Encoder did not change. Check your wiring.")
        return

    degrees_per_tick = known_degrees / ticks
    enc_sign = 1 if degrees_per_tick > 0 else -1
    degrees_per_tick_abs = abs(degrees_per_tick)

    print(f"\n=== RESULTS ===")
    print(f"  Ticks traveled    : {ticks:.4f}")
    print(f"  Degrees traveled  : {known_degrees:.2f}")
    print(f"  DEGREES_PER_TICK  : {degrees_per_tick_abs:.6f}")
    print(f"  ENC_SIGN          : {enc_sign}")
    print(f"\nCopy these into your motor code:")
    print(f"  DEGREES_PER_TICK = {degrees_per_tick_abs:.6f}")
    print(f"  ENC_SIGN         = {enc_sign}")

    # Step 4: verify by running the motor
    verify = input("\nWould you like to verify by running the motor? (y/n): ")
    if verify.strip().lower() != 'y':
        return

    test_degrees = float(input("Enter degrees to test (e.g. 45): "))
    ticks_needed  = test_degrees / degrees_per_tick_abs
    target_enc    = link1_motor.position + (enc_sign * ticks_needed)
    direction     = 1 if target_enc > link1_motor.position else -1

    print(f"\nRunning motor for {test_degrees} degrees...")
    MOTOR_POWER = 0.3
    TOLERANCE   = 1.0

    while True:
        curr_enc     = link1_motor.position
        ticks_left   = enc_sign * (target_enc - curr_enc)
        degrees_left = ticks_left * degrees_per_tick_abs

        print(f"  Encoder: {curr_enc:.2f}  |  Degrees left: {degrees_left:.2f}")

        if abs(degrees_left) <= TOLERANCE:
            link1_motor.power_command = 0
            break

        link1_motor.power_command = direction * MOTOR_POWER
        time.sleep(0.01)

    actual_ticks   = enc_sign * (link1_motor.position - start_enc - ticks_needed)
    actual_degrees = link1_motor.position * degrees_per_tick_abs
    print(f"\nMotor stopped.")
    print(f"  Manually measure how far the arm actually moved.")
    print(f"  If it is not {test_degrees} degrees, adjust DEGREES_PER_TICK up or down.")


# ── Main ───────────────────────────────────────────────────────────────────────
try:
    calibrate()
except KeyboardInterrupt:
    print("\nInterrupted.")
finally:
    link1_motor.power_command = 0
    print("Motor stopped.")