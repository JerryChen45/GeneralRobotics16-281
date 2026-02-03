import time
import math
from motorgo.plink import Plink, ControlMode

# ================= INITIALIZE =================
plink = Plink()
plink.connect()

left_motor = plink.channel1
right_motor = plink.channel2

left_motor.control_mode = ControlMode.POWER
right_motor.control_mode = ControlMode.POWER

# Ensure motors are off
left_motor.power_command = 0
right_motor.power_command = 0

# ========== INITIAL GUESSES ==========
WHEEL_RADIUS = 1.0      # inches
WHEELBASE = 5.0         # inches
TICKS_PER_REV = 12
# ====================================


def calibrate_ticks_per_rev():
    """Find encoder ticks per wheel revolution (manual wheel turn)"""
    print("\n" + "=" * 60)
    print("CALIBRATION 1: TICKS PER REVOLUTION")
    print("=" * 60)

    print("\nMotors are OFF.")
    print("Put tape on ONE wheel.")
    print("Rotate the wheel EXACTLY one full revolution.\n")

    input("Press Enter to record starting encoder value...")

    start = left_motor.position
    print(f"Initial encoder value: {start}")

    input("Rotate wheel ONE full turn, then press Enter...")

    end = left_motor.position
    ticks = abs(end - start)

    print(f"\nFinal encoder value: {end}")
    print(f"Measured ticks: {ticks:.2f}")

    if ticks < 1:
        print("ERROR: No encoder movement detected.")
        return None

    print("\n" + "=" * 60)
    print(f">>> TICKS_PER_REV = {ticks:.2f}")
    print("=" * 60 + "\n")

    return ticks


def calibrate_wheel_radius(ticks_per_rev):
    """Find wheel radius by driving straight and measuring distance"""
    print("\n" + "=" * 60)
    print("CALIBRATION 2: WHEEL RADIUS")
    print("=" * 60)

    input("\nPress Enter to begin...")

    left_start = left_motor.position
    right_start = right_motor.position

    print("\nDriving in 2 seconds...")
    time.sleep(2)

    left_motor.power_command = 0.5
    right_motor.power_command = 0.5
    time.sleep(3.0)

    left_motor.power_command = 0
    right_motor.power_command = 0

    left_end = left_motor.position
    right_end = right_motor.position

    left_ticks = abs(left_end - left_start)
    right_ticks = abs(right_end - right_start)
    avg_ticks = (left_ticks + right_ticks) / 2.0

    print(f"\nLeft ticks:  {left_ticks:.1f}")
    print(f"Right ticks: {right_ticks:.1f}")
    print(f"Average:     {avg_ticks:.1f}")

    if avg_ticks < 1:
        print("ERROR: Not enough encoder movement.")
        return None

    measured_dist = float(input("\nEnter distance traveled (inches): "))

    revolutions = avg_ticks / ticks_per_rev
    circumference = measured_dist / revolutions
    wheel_radius = circumference / (2 * math.pi)

    print("\n" + "=" * 60)
    print(f">>> WHEEL_RADIUS = {wheel_radius:.3f} inches")
    print("=" * 60 + "\n")

    return wheel_radius


def measure_wheelbase():
    print("\n" + "=" * 60)
    print("CALIBRATION 3: WHEELBASE")
    print("=" * 60)

    print("\nMeasure center-to-center wheel distance.")
    wheelbase = float(input("Enter wheelbase (inches): "))

    print("\n" + "=" * 60)
    print(f">>> WHEELBASE = {wheelbase:.3f} inches")
    print("=" * 60 + "\n")

    return wheelbase


def test_minimum_speed(radius, wheelbase, ticks_per_rev):
    print("\n" + "=" * 60)
    print("TEST: MINIMUM SPEED REQUIREMENT")
    print("=" * 60)

    input("\nPress Enter to start test...")

    x = y = theta = 0.0
    prev_left = left_motor.position
    prev_right = right_motor.position

    print("\nDriving at full power in 2 seconds...")
    time.sleep(2)

    left_motor.power_command = 1.0
    right_motor.power_command = 1.0

    start_time = time.time()
    while time.time() - start_time < 3.0:
        curr_left = left_motor.position
        curr_right = right_motor.position

        dL = curr_left - prev_left
        dR = curr_right - prev_right

        left_dist = (dL / ticks_per_rev) * (2 * math.pi * radius)
        right_dist = (dR / ticks_per_rev) * (2 * math.pi * radius)

        center_dist = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / wheelbase

        x += center_dist * math.cos(theta)
        y += center_dist * math.sin(theta)
        theta += delta_theta

        prev_left = curr_left
        prev_right = curr_right
        time.sleep(0.01)

    left_motor.power_command = 0
    right_motor.power_command = 0

    distance = math.sqrt(x**2 + y**2)

    print("\n" + "=" * 60)
    print(f"Distance traveled: {distance:.2f} inches")
    print("Required minimum:  12.00 inches")

    if distance >= 12:
        print("\n✓ PASS")
    else:
        print("\n✗ FAIL")

    print("=" * 60 + "\n")


def main():
    while True:
        print("\n" + "=" * 60)
        print("CALIBRATION MENU")
        print("=" * 60)
        print("1. Full calibration")
        print("2. Calibrate ticks per rev")
        print("3. Calibrate wheel radius")
        print("4. Measure wheelbase")
        print("5. Test minimum speed")
        print("6. Exit")

        choice = input("\nChoice (1–6): ")

        if choice == "1":
            t = calibrate_ticks_per_rev()
            if t:
                r = calibrate_wheel_radius(t)
                b = measure_wheelbase()
                test_minimum_speed(r, b, t)
        elif choice == "2":
            calibrate_ticks_per_rev()
        elif choice == "3":
            calibrate_wheel_radius(float(input("TICKS_PER_REV: ")))
        elif choice == "4":
            measure_wheelbase()
        elif choice == "5":
            test_minimum_speed(
                float(input("WHEEL_RADIUS: ")),
                float(input("WHEELBASE: ")),
                float(input("TICKS_PER_REV: "))
            )
        elif choice == "6":
            break
        else:
            print("Invalid choice.")


if __name__ == "__main__":
    try:
        main()
    finally:
        left_motor.power_command = 0
        right_motor.power_command = 0
