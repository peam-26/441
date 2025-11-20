################################################################################
# WEB + STEPPER MOTORS + LASER CONTROL (LIVE SLIDERS, -180..180)
#
# - Two 28BYJ-48 steppers via 74HC595 shift register (Shifter class)
# - Web UI (port 8080) with:
#       * Slider for Motor 1 (Azimuth): -180 to +180
#       * Slider for Motor 2 (Altitude): -180 to +180
#       * Live updates while sliding (AJAX)
#       * Test Laser (3s) button
################################################################################

import time
import multiprocessing
import socket
import threading
from shifter import Shifter
import RPi.GPIO as GPIO

# --------------------------- GPIO / GLOBALS -----------------------------------

GPIO.setmode(GPIO.BCM)
LASER_PIN = 17
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)

# Shared array for 2 motors (each uses 4 bits in a nibble)
myArray = multiprocessing.Array('i', 2)


# ------------------------------ STEPPER CLASS ---------------------------------
class Stepper:
    # Half-step sequence for 28BYJ-48
    seq = [0b0001, 0b0011, 0b0010, 0b0110,
           0b0100, 0b1100, 0b1000, 0b1001]

    delay = 3000                      # microseconds between steps
    steps_per_degree = 2048 / 360.0   # 28BYJ-48 typical

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock      # shared lock (both motors share same shift register)
        self.index = index    # 0 or 1
        self.angle = 0.0      # current angle in degrees, kept in [-180, 180]
        self.step_state = 0   # index into seq[]
        self.shifter_bit_start = 4 * index  # which nibble in the byte

    def _sgn(self, x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0

    def _normalize_angle(self, a):
        """Keep angle in [-180, 180]."""
        while a > 180:
            a -= 360
        while a < -180:
            a
