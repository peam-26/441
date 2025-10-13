from shifter import Shifter

import RPi.GPIO as GPIO
import time
import random


s = Shifter(serial=23, latch=24, clock=25)

pos = 0  # starting led position (1st led)

try:
    while True:
        s.shiftByte(1 << pos)
        time.sleep(.05)

        step = random.choice((-1, 1))
        nxtpos = pos + step

        # Bounce if you hit the edges (keep inside 0–7)
        if nxtpos < 0 or nxtpos >= 8:
            step = -step
            nxtpos = pos + step

        pos = nxtpos

except:
    s.shiftByte(0)  
    GPIO.cleanup()
