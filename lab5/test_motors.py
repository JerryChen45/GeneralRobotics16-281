"""
File:           test_motors.py
Description:    Quick motor test for verifying Plink hardware before demo.
                Tests: forward, backward, turn left, turn right.
"""

import time
from motorgo.plink import Plink, ControlMode

# ========== SETUP ==========
plink = Plink()
plink.connect()

left_motor = plink.channel1
right_motor = plink.channel3

left_motor.control_mode = ControlMode.POWER
right_motor.control_mode = ControlMode.POWER

left_motor.power_command = 0
right_motor.power_command = 0

# Motor direction conventions (from Lab 3)
LEFT_DIR = -1
RIGHT_DIR = 1

POWER = 0.4
DURATION = 2.0  # seconds per test


def stop():
    left_motor.power_command = 0
    right_motor.power_command = 0


def test(name, left_power, right_power):
    print(f"\n--- {name} ---")
    input(f"Press Enter to run (power L={left_power:.1f}, R={right_power:.1f})...")
    left_motor.power_command = LEFT_DIR * left_power
    right_motor.power_command = RIGHT_DIR * right_power
    time.sleep(DURATION)
    stop()
    print("Done.")


if __name__ == "__main__":
    print("Motor Test - Plink")
    print(f"Power: {POWER}, Duration: {DURATION}s per test")
    print("Press Ctrl+C at any time to stop.\n")

    try:
        test("FORWARD", POWER, POWER)
        test("BACKWARD", -POWER, -POWER)
        test("TURN LEFT (CCW)", -POWER, POWER)
        test("TURN RIGHT (CW)", POWER, -POWER)
        print("\nAll tests complete!")
    except KeyboardInterrupt:
        stop()
        print("\nStopped.")
