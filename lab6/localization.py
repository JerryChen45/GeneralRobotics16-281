import time
import board
import adafruit_vl53l4cx
from motorgo import Plink, ControlMode

def main():
    i2c = board.I2C()  # uses board.SCL and board.SDA

    dist = adafruit_vl53l4cx.VL53L4CX(i2c)

    print("VL53L4CX Simple Test.")

    dist.start_ranging()
    
    while True:
        dist.clear_interrupt()
        print("Distance: {} cm".format(dist.distance))





