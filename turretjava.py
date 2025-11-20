################################################################################
# WEB + STEPPER MOTORS + LASER CONTROL (LIVE SLIDERS, -180..180, DEBUG)
################################################################################

import time
import multiprocessing
import socket
import threading
from shifter import Shifter          # make sure shifter.py defines class Shifter
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
            a += 360
        return a

    def _step(self, direction):
        with self.lock:
            # Update sequence index
            self.step_state = (self.step_state + direction) % 8

            # Put this motor's 4-bit pattern into its nibble
            myArray[self.index] &= ~(0b1111 << self.shifter_bit_start)
            myArray[self.index] |= (Stepper.seq[self.step_state] << self.shifter_bit_start)

            # Combine all nibble values into a single byte
            final = 0
            for val in myArray:
                final |= val

            # Shift out to 74HC595
            self.s.shiftByte(final)

        # Update angle (outside the lock)
        self.angle += direction / Stepper.steps_per_degree
        self.angle = self._normalize_angle(self.angle)

        time.sleep(Stepper.delay / 1e6)

    def _rotate(self, delta):
        direction = self._sgn(delta)
        steps = int(abs(delta) * Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    def rotate(self, delta):
        """
        Relative rotation by `delta` degrees (can be positive or negative).
        Runs in a separate process so the HTTP handler doesn't block.
        """
        if delta == 0:
            return None
        p = multiprocessing.Process(target=self._rotate, args=(delta,))
        p.daemon = True
        p.start()
        return p

    def goAngle(self, target):
        """
        Absolute move to `target` degrees in [-180, 180], using shortest path.
        """
        target = max(-180.0, min(180.0, float(target)))
        current = self.angle
        delta = target - current
        # Clamp if someone sends something crazy
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        return self.rotate(delta)

    def zero(self):
        self.angle = 0.0


# ----------------------------- LASER CONTROL ----------------------------------
def test_laser():
    GPIO.output(LASER_PIN, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(LASER_PIN, GPIO.LOW)


# ----------------------------- HTTP HELPERS -----------------------------------
def parsePOSTdata(data):
    """
    Very simple x-www-form-urlencoded parser for the POST body.
    """
    data_dict = {}
    idx = data.find('\r\n\r\n')
    if idx == -1:
        return data_dict
    post = data[idx + 4:]
    pair
