import RPi.GPIO as GPIO
import time
import math

GPIO.setmode(GPIO.BCM)

pins = [27,22,10,9,11,5,6,13,19,26] #10 leds

for i in pins:
	GPIO.setup(i, GPIO.OUT) #setup as outputs

f = .2
base = 500
phi = math.pi/11

pwms = [GPIO.PWM(i,base) for i in pins]

for j in pwms:
	j.start(0)

t0 = time.time()

try:
	while True:
		t = time.time() - t0 #current time elapsed
		for i in range(10):
			pwm = pwms[i]
			B = math.sin(2*math.pi*f*t - (i*phi))**2 #whole B function, including the pi/11 phase subtracted from each level
			pwm.ChangeDutyCycle(B*100)

except KeyboardInterrupt:
	pass

finally:
	for j in pwms:
		j.stop()
	GPIO.cleanup()


