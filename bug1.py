from shifter import Shifter

import RPi.GPIO as GPIO
import time
import random
import threading  # ADDED

LED_COUNT = 8  # ADDED


# ADDED: wrap existing behavior into a Bug class with required attributes and methods
class Bug:
    def __init__(self, timestep=0.1, x=3, isWrapOn=False, *, serial=23, latch=24, clock=25):
        self.timestep = float(timestep)     # ADDED
        self.x = int(x) % LED_COUNT         # ADDED
        self.isWrapOn = bool(isWrapOn)      # ADDED

        self.__shifter = Shifter(serial=serial, latch=latch, clock=clock)  # ADDED (private)
        self.__stop_evt = threading.Event()  # ADDED
        self.__thread = None                 # ADDED

    def __run(self):  # ADDED (internal loop using your original logic)
        pos = self.x
        try:
            while not self.__stop_evt.is_set():
                self.__shifter.shiftByte(1 << pos)
                time.sleep(self.timestep)

                step = random.choice((-1, 1))
                nxtpos = pos + step

                # Bounce if you hit the edges (keep inside 0–7) — SAME LOGIC
                if self.isWrapOn:
                    nxtpos %= LED_COUNT
                else:
                    if nxtpos < 0 or nxtpos >= LED_COUNT:
                        step = -step
                        nxtpos = pos + step

                pos = nxtpos
                self.x = pos  # keep x updated
        finally:
            pass

    def start(self):  # ADDED
        if self.__thread and self.__thread.is_alive():
            return
        self.__stop_evt.clear()
        self.__thread = threading.Thread(target=self.__run, daemon=True)
        self.__thread.start()

    def stop(self):  # ADDED
        self.__stop_evt.set()
        if self.__thread:
            self.__thread.join(timeout=self.timestep * 2 + 0.1)
        self.__shifter.shiftByte(0)
        GPIO.cleanup()


# OPTIONAL: quick usage (no main guard since you asked to avoid it)
# Comment these three lines out if you don’t want it to auto-run.
bug = Bug(timestep=0.05, x=0, isWrapOn=False)  # ADDED
bug.start()                                     # ADDED
# Press Ctrl+C to stop; or call bug.stop() from elsewhere.
