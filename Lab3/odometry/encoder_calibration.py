import time
from motorgo.plink import Plink, ControlMode

# ========== CONFIG ==========
CHANNEL = 4         # CHANGE to 1 (left) or 3 (right)
MOTOR_DIR = -1.0     # match physical motor direction
POWER = 0.2          # set to 0.0 if spinning by hand
# ============================

plink = Plink()
plink.connect()

motor = getattr(plink, f"channel{CHANNEL}")
motor.control_mode = ControlMode.POWER
motor.power_command = 0.0

time.sleep(0.5)

print("\nENCODER CALIBRATION")
print("------------------")
print("1 full wheel revolution")
print("Press ENTER to start...")
input()

# Zero encoder
start_pos = motor.position
print("Encoder zeroed.")

# Start motor (optional)
motor.power_command = MOTOR_DIR * POWER

print("Rotate wheel ONE full revolution.")
print("Press ENTER exactly when the wheel completes 1 turn.")
input()

# Stop motor
motor.power_command = 0.0
time.sleep(0.2)

end_pos = motor.position
ticks = abs(end_pos - start_pos)

print("\n==============================")
print(f"Total encoder ticks: {ticks:.3f}")
print("This is your TICKS_PER_REV")
print("==============================")

