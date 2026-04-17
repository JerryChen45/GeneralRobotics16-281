import time

from motorgo import ControlMode, Plink

# ── Setup ──────────────────────────────────────────────────────────────────────
plink = Plink()

link2_motor = plink.channel3   # joint 2 is on channel 3

plink.connect()

link2_motor.control_mode = ControlMode.POWER
link2_motor.power_command = 0

# ── Conversion factor ─────────────────────────────────────────────────────────
DEGREES_PER_TICK = 19
ENC_SIGN         = -1

# ── Zero the encoder at current position ──────────────────────────────────────
print("=== ENCODER TEST — JOINT 2 ===")
print(f"Using DEGREES_PER_TICK={DEGREES_PER_TICK}  ENC_SIGN={ENC_SIGN}")
input("Position joint 2 at 0 degrees, then press ENTER to zero.")
zero = link2_motor.position
print(f"Zeroed at encoder={zero:.4f}")
print("Turn the link by hand. Degrees will print in real time.")
print("Press Ctrl+C to stop.\n")

try:
    while True:
        raw   = link2_motor.position
        ticks = ENC_SIGN * (raw - zero)
        deg   = ticks * DEGREES_PER_TICK
        print(f"  encoder: {raw:10.4f}   degrees: {deg:8.2f}°", end="\r")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n\nStopped.")
finally:
    link2_motor.power_command = 0
