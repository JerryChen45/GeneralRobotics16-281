import time
import board
import adafruit_vl53l4cx
from motorgo import Plink, ControlMode

sector_num = 16
map = [1,1,0,1,0,1,1,0,0,0,0,0,0,0,0,0]
particles_num = 3200
convergence_thresh = 0.7


def main():
    i2c = board.I2C()

    dist = adafruit_vl53l4cx.VL53L4CX(i2c)

    print("VL53L4CX Simple Test.")

    dist.start_ranging()
    
    while True:
        dist.clear_interrupt()
        print("Distance: {} cm".format(dist.distance))

if __name__ == "__main__":
    main()





