import time
from motorgo.plink import Plink
from motorgo import ControlMode

# Initialize and connect
p = Plink()
p.connect()

# Motor channels (change if needed)
left = p.channel1
right = p.channel3

# Set control mode
left.control_mode = ControlMode.POWER
right.control_mode = ControlMode.POWER

print("Starting encoder test...")
print("Motors at (1, 1). Press Ctrl+C to stop.\n")

# Command both motors
left.power_command = -1.0
right.power_command = 1.0

try:
    while True:
        left_pos = left.position
        right_pos = right.position

        print(f"Left encoder: {left_pos:8.2f}   Right encoder: {right_pos:8.2f}")
        time.sleep(0.1)   # 10 Hz printing

except KeyboardInterrupt:
    pass

finally:
    # Stop motors
    left.power_command = 0.0
    right.power_command = 0.0
    print("\nStopped.")
