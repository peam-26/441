from shifter import Shifter

import RPi.GPIO as GPIO
import time
import random
GPIO.setmode(GPIO.BCM)


class Bug:
    def __init__(self, timestep=0.1, x=3, isWrapOn=False):
        self.timestep = timestep
        self.x = x      
        self.isWrapOn = isWrapOn  
        self.__shifter = Shifter(23, 24, 25)
        self.active = False

    def start(self):
        self.active = True

    def stop(self):
        self.active = False
        self.__shifter.shiftByte(0)

    def step(self, timestep):
        if not self.active:
            time.sleep(timestep)
            return

        self.__shifter.shiftByte(1 << self.x) # show current position
        step = 1 if random.getrandbits(1) else -1
        nxt  = self.x + step


        if self.isWrapOn:
            self.x = nxt % 8
        else:
            #if outside bounds, reverse the step
            if not (0 <= nxt < 8):
                self.x = self.x - step
            else:
                self.x = nxt

        time.sleep(timestep)


s1, s2, s3 = 10, 9, 11
for pin in [s1, s2, s3]:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

bug = Bug()
def toggle_wrap(channel):
    bug.isWrapOn = not bug.isWrapOn
    print("Wrap:", bug.isWrapOn)

GPIO.add_event_detect(s2, GPIO.BOTH, callback=toggle_wrap, bouncetime=300)

try:
    while True:
        if GPIO.input(s1):
            if not bug.active:
                bug.start()
        else:
            if bug.active:
                bug.stop()

        quick = bug.timestep / 3.0 if GPIO.input(s3) else bug.timestep
        bug.step(quick)


except KeyboardInterrupt:
    pass
finally:
    bug.stop()
    GPIO.cleanup()
