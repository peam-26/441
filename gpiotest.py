import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

PIN = 17

GPIO.setup(PIN, GPIO.OUT)
GPIO.output(PIN, GPIO.HIGH)
time.sleep(1)
GPIO.output(PIN, GPIO.LOW)

GPIO.cleanup()
