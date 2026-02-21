import time
import sys
import board
import adafruit_bh1750
from motorgo import Plink, ControlMode

left_ch = 1
right_ch = 4

base_power = 0.5

target_lux = 60
kp = 0.4
kd = 0.1
dt = 0.01


def main():
    i2c = board.I2C()
    light = adafruit_bh1750.BH1750(i2c)

    p = Plink(frequency=200, timeout=1.0)
    left = None
    right = None

    try:
        print("Connecting...")
        p.connect()
        print("Connected.")

        left = getattr(p, f"channel{left_ch}")
        right = getattr(p, f"channel{right_ch}")

        left.control_mode = ControlMode.POWER
        right.control_mode = ControlMode.POWER

        left.power_command = 0.0
        right.power_command = 0.0
        time.sleep(0.2)
        prev_error = 0


        while True:
            lux = float(light.lux)
            error = lux - target_lux
            steer = kp * error + kd * (error - prev_error) / dt
            right_pwr = base_power + steer
            left_pwr = base_power - steer

            right.power_command = max(-1.0, min(1.0, right_pwr))
            left.power_command = max(-1.0, min(1.0, left_pwr))

            prev_error = error

            time.sleep(dt)
    except:
        print("Sensor Not working!!")