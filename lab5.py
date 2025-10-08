import RPi.GPIO as GPIO
import time
import math

GPIO.setmode(GPIO.BCM)

pins = [27,22,10,9,11,5,6,13,19,26] #10 leds

for i in pins:
	GPIO.setup(i, GPIO.OUT) #setup as outputs

GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #pin 21 is the pin to swtich between 3.3V and gnd, enabling pulldown here

f = .2
base = 500
phi = math.pi/11

pwms = [GPIO.PWM(i,base) for i in pins]

for j in pwms:
	j.start(0)

t0 = time.time()

direction = 1
def reverse(channel):
    global direction
    direction *= -1

GPIO.add_event_detect(21, GPIO.BOTH, callback=reverse, bouncetime=300)

try:
	while True:
		t = time.time() - t0 #current time elapsed
		for i in range(10):
			pwm = pwms[i]
			B = math.sin(2*math.pi*f*t - (direction*i*phi))**2 #whole B function, including the pi/11 phase subtracted from each level
			pwm.ChangeDutyCycle(B*100)

except KeyboardInterrupt:
	pass

finally:
	for j in pwms:
		j.stop()
	GPIO.cleanup()


