import math
import time
import numpy as np

# global constants
WHEEL_RADIUS = 1.125 
WHEELBASE = 6.5
TICKS_PER_REV = 6.4845
DT = 0.01
LEFT_MOTOR_DIR = -1
RIGHT_MOTOR_DIR = 1

def angular_velocity(initial_c, previous_c, time_step, ticks_per_rev):
    delta_c = initial_c - previous_c
    omega = delta_c / time_step


def rk():
    omega = 




