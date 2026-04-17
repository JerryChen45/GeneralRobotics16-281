import time
from motorgo import ControlMode, Plink


def main():
    plink = Plink()
    left_motor  = plink.channel1
    right_motor = plink.channel4
    plink.connect()

    left_motor.control_mode  = ControlMode.POWER
    right_motor.control_mode = ControlMode.POWER

    left_motor.power_command  =  0.5
    right_motor.power_command = -0.5

    print("Motors running. Printing encoder values (Ctrl+C to stop)...\n")
    try:
        while True:
            print(f"Left  (ch1): pos={left_motor.position:.4f}  vel={left_motor.velocity:.4f}")
            print(f"Right (ch4): pos={right_motor.position:.4f}  vel={right_motor.velocity:.4f}")
            print("----")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        left_motor.power_command  = 0
        right_motor.power_command = 0
        print("\nStopped.")


if __name__ == "__main__":
    main()
