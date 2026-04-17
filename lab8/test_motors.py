# spin_motors.py
# Before running this script, ensure that the MotorGo Plink is
# connected to the Raspberry Pi and that it has been flashed with the
# MotorGo firmware.

import time

from motorgo import BrakeMode, ControlMode, Plink


def main():
    # Create a Plink object, the main interface to the MotorGo board
    plink = Plink()

    # The Plink object has 4 MotorChannel objects, corresponding to the 4 motor channels
    # on the board
    # You can access them directly: plink.channel1
    # Or you can save references as local variables for convenience:
    left_motor = plink.channel1
    right_motor = plink.channel4

    # Finally, connect to the MotorGo board and push the configuration
    plink.connect()

    # You can configure how you want to control the motor channels.
    # Power mode: Set the power of the motor in the range [-1, 1]
    #            This directly corresponds to setting a voltage to the motor
    #
    # Velocity mode: Set the velocity of the motor in rad/s
    #              This mode requires setting the velocity PID gains
    #              It also requires an encoder to be connected to the motor
    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    try:
        while True:
            left_motor.power_command = 1
            right_motor.power_command = -1
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        left_motor.power_command = 0
        right_motor.power_command = 0
        print("\nMotors stopped.")


if __name__ == "__main__":
    main()
