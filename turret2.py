################################################################################
# WEB + STEPPER MOTORS + LASER CONTROL (COMBINED VERSION)
#
# - Two 28BYJ-48 steppers via 74HC595 shift register (Shifter class)
# - Simple HTML page (port 8080) to:
#       * Set Motor 1 angle (degrees)
#       * Set Motor 2 angle (degrees)
#       * Test laser for 3 seconds
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

# Shared array for 2 motors (each uses 4 bits)
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
        self.lock = lock      # shared lock (both motors use same lock)
        self.index = index    # 0 or 1
        self.angle = 0.0      # current angle in degrees
        self.step_state = 0   # index into seq[]
        self.shifter_bit_start = 4 * index  # which nibble in the byte

    def _sgn(self, x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0

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
        self.angle = (self.angle + direction / Stepper.steps_per_degree) % 360.0
        time.sleep(Stepper.delay / 1e6)

    def _rotate(self, delta):
        direction = self._sgn(delta)
        steps = int(abs(delta) * Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    def rotate(self, delta):
        # Run rotation in a separate process so HTTP handler doesn’t block
        p = multiprocessing.Process(target=self._rotate, args=(delta,))
        p.start()
        return p

    def goAngle(self, target):
        # Move via shortest path
        # Normalize to 0–360
        target = target % 360.0
        current = self.angle % 360.0
        delta = (target - current + 540.0) % 360.0 - 180.0
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
    pairs = post.split('&')
    for p in pairs:
        if '=' in p:
            key, val = p.split('=', 1)
            data_dict[key] = val
    return data_dict


def web_page(m1_angle, m2_angle):
    """
    Simple HTML UI for controlling both steppers and testing the laser.
    """
    html = f"""
    <html>
    <head>
        <title>Turret Control</title>
    </head>
    <body style="font-family: Arial; text-align:center; margin-top:40px;">
        <h2>Stepper Motor + Laser Control</h2>

        <form action="/" method="POST">
            <h3>Motor 1 (Azimuth)</h3>
            <input type="number" name="m1" value="{m1_angle:.1f}" step="1" min="0" max="360">
            <br><br>

            <h3>Motor 2 (Altitude)</h3>
            <input type="number" name="m2" value="{m2_angle:.1f}" step="1" min="0" max="360">
            <br><br>

            <input type="submit" value="Rotate Motors">
            <br><br>

            <button type="submit" name="laser_test" value="1">
                Test Laser (3s)
            </button>
        </form>
    </body>
    </html>
    """
    return html.encode('utf-8')


# ------------------------------- WEB SERVER -----------------------------------
def serve_web(m1, m2):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 8080))
    s.listen(3)
    print("Web server running on port 8080...")

    try:
        while True:
            conn, addr = s.accept()
            try:
                msg = conn.recv(4096).decode('utf-8', errors='ignore')
                if not msg:
                    conn.close()
                    continue

                if msg.startswith("POST"):
                    data = parsePOSTdata(msg)

                    # Motor 1
                    if "m1" in data and data["m1"].strip() != "":
                        try:
                            m1_target = float(data["m1"])
                            p = m1.goAngle(m1_target)
                            # Don't join, let it run in background
                        except ValueError:
                            pass

                    # Motor 2
                    if "m2" in data and data["m2"].strip() != "":
                        try:
                            m2_target = float(data["m2"])
                            p = m2.goAngle(m2_target)
                        except ValueError:
                            pass

                    # Laser test
                    if "laser_test" in data:
                        threading.Thread(target=test_laser, daemon=True).start()

                # Always respond with current angles
                response_body = web_page(m1.angle, m2.angle)
                header = b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
                conn.sendall(header + response_body)

            finally:
                conn.close()
    finally:
        s.close()


# ----------------------------------- MAIN -------------------------------------
if __name__ == '__main__':
    # One shared shifter for both motors
    s = Shifter(23, 24, 25)

    # One shared lock for both motors, since they share the shift register
    shared_lock = multiprocessing.Lock()

    m1 = Stepper(s, shared_lock, 0)   # Motor 1 (azimuth)
    m2 = Stepper(s, shared_lock, 1)   # Motor 2 (altitude)

    m1.zero()
    m2.zero()

    # Start web server thread
    t = threading.Thread(target=serve_web, args=(m1, m2), daemon=True)
    t.start()

    print("Motors initialized. Web interface ready.")
    print("Open a browser to: http://<raspberry-pi-ip>:8080")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        GPIO.cleanup()
