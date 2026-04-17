import math
import time

from motorgo import ControlMode, Plink

# ── Robot constants ────────────────────────────────────────────────────────────
DIST_PER_TICK = 0.0569  # calibrated distance per encoder tick
ENC_SIGN      = -1         # flip to +1 if distance counts backward
DT            = 0.01       # seconds between odometry updates
MOTOR_POWER   = 0.5        # drive power [0, 1]
THETA         = 0.0        # heading in radians; 0 = along +X axis


# ── Odometry update ────────────────────────────────────────────────────────────
def update_odometry(state, prev_enc, motor):
    curr_enc    = motor.position
    delta_ticks = ENC_SIGN * (curr_enc - prev_enc)
    dist        = delta_ticks * DIST_PER_TICK

    state[0] += dist * math.cos(THETA)
    state[1] += dist * math.sin(THETA)

    return state, curr_enc


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    target_distance = float(input("Enter target distance: "))
    print(f"\nTarget: {target_distance:.2f} units. Starting...\n")

    plink = Plink()

    left_motor  = plink.channel1
    right_motor = plink.channel4

    plink.connect()

    left_motor.control_mode  = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    state    = [0.0, 0.0]
    prev_enc = right_motor.position

    try:
        while True:
            state, prev_enc = update_odometry(state, prev_enc, right_motor)

            distance_traveled = math.sqrt(state[0]**2 + state[1]**2)

            print(
                f"Encoder : {right_motor.position:.2f}  |  "
                f"Traveled : {distance_traveled:.4f}  /  {target_distance:.4f}"
            )

            if distance_traveled >= target_distance:
                left_motor.power_command  = 0
                right_motor.power_command = 0
                print(f"\nTarget reached! Stopped at {distance_traveled:.4f} units.")
                break

            left_motor.power_command  =  MOTOR_POWER
            right_motor.power_command = -MOTOR_POWER

            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        left_motor.power_command  = 0
        right_motor.power_command = 0
        distance_traveled = math.sqrt(state[0]**2 + state[1]**2)
        print(f"Final distance traveled → {distance_traveled:.4f}")


if __name__ == "__main__":
    main()