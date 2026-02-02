import time
import board
import adafruit_bh1750

TARGET_LUX = 68.0
HYSTERESIS = 3.0
DT = 0.05

def main():
    i2c = board.I2C()
    light = adafruit_bh1750.BH1750(i2c)

    state = "UNKNOWN"

    print("BH1750 LUX monitor starting.")
    print(f"TARGET_LUX={TARGET_LUX:.1f} HYSTERESIS=±{HYSTERESIS:.1f} DT={DT:.2f}s")
    print("Move the sensor over white and black surfaces. Ctrl+C to stop.\n")

    try:
        while True:
            lux = float(light.lux)

            if lux > TARGET_LUX + HYSTERESIS:
                state = "WHITE"
            elif lux < TARGET_LUX - HYSTERESIS:
                state = "BLACK"

            print(f"lux={lux:7.1f} -> {state}")
            time.sleep(DT)

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
