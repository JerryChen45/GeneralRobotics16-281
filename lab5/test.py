import time
from motorgo import ControlMode, Plink


def main():
    plink = Plink()

    left_motor = plink.channel1
    right_motor = plink.channel4

    plink.connect()

    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    # Drive both motors forward at low power for 2 seconds
    left_motor.power_command = 0.6
    right_motor.power_command = -0.6  # negative because right motor is flipped

    try:
        for i in range(200):  # 2 seconds at 0.01s intervals
            print(f"L_pos: {left_motor.position:>8.2f}  L_vel: {left_motor.velocity:>8.2f}  |  "
                  f"R_pos: {right_motor.position:>8.2f}  R_vel: {right_motor.velocity:>8.2f}")
            time.sleep(0.01)
    finally:
        left_motor.power_command = 0
        right_motor.power_command = 0
        print("\nMotors stopped.")


if __name__ == "__main__":
    main()