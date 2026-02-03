from motorgo import MotorGo

mg = MotorGo()

# Read initial encoder value
initial = mg.get_encoder_count(0)
print(f"Initial encoder value: {initial}")

# Now manually rotate the wheel EXACTLY one full revolution
# - Mark a spot on your wheel with tape
# - Rotate it until the tape comes back to the same position
input("Rotate the wheel exactly ONE full revolution, then press Enter...")

# Read final encoder value
final = mg.get_encoder_count(0)
print(f"Final encoder value: {final}")

# Calculate ticks per revolution
ticks_per_rev = abs(final - initial)
print(f"\nTICKS_PER_REV = {ticks_per_rev}")