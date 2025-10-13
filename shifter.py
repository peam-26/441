import RPi.GPIO as GPIO
import time

class Shifter:
    def __init__(self, serial, latch, clock):
        self.dataPin = serial
        self.latchPin = latch
        self.clockPin = clock

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dataPin, GPIO.OUT)
        GPIO.setup(self.latchPin, GPIO.OUT, initial=0)  # start latch & clock low
        GPIO.setup(self.clockPin, GPIO.OUT, initial=0)

    def ping(self, p):  # based on public ping() from lecture
        GPIO.output(p, 1)
        time.sleep(0)
        GPIO.output(p, 0)

    def shiftByte(self, b):  # send a byte of data to the output
        for i in range(8):
            GPIO.output(self.dataPin, b & (1 << i))
            self.ping(self.clockPin)  # add bit to register
        self.ping(self.latchPin)  # send register to output


try:
    test = Shifter(23, 24, 25)
    while 1:
        for i in range(2**8):
            test.shiftByte(i)
            time.sleep(0.5)
except:
    GPIO.cleanup()
