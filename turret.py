import time
import multiprocessing
import socket
import threading
from shifter import Shifter
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
LASER_PIN = 17
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)

myArray = multiprocessing.Array('i', 2)

class Stepper:
    seq = [0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001]
    delay = 3000 
    steps_per_degree = 2048 / 360

    def __init__(self, shifter, lock, index):
        self.s = shifter
        self.lock = lock
        self.index = index
        self.angle = 0
        self.step_state = 0
        self.shifter_bit_start = 4 * index

    def _sgn(self, x):
        return 0 if x == 0 else int(abs(x) / x)

    def _step(self, direction):
        with self.lock:
            self.step_state = (self.step_state + direction) % 8

            myArray[self.index] &= ~(0b1111 << self.shifter_bit_start)
            myArray[self.index] |= (Stepper.seq[self.step_state] << self.shifter_bit_start)

            final = 0
            for val in myArray:
                final |= val

            self.s.shiftByte(final)

        self.angle = (self.angle + direction / Stepper.steps_per_degree) % 360
        time.sleep(Stepper.delay / 1e6)

    def _rotate(self, delta):
        direction = self._sgn(delta)
        steps = int(abs(delta) * Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    def rotate(self, delta):
        p = multiprocessing.Process(target=self._rotate, args=(delta,))
        p.start()
        return p

    def goAngle(self, target):
        delta = (target - self.angle + 540) % 360 - 180
        return self.rotate(delta)

    def zero(self):
        self.angle = 0

def test_laser():
    GPIO.output(LASER_PIN, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(LASER_PIN, GPIO.LOW)

def parsePOSTdata(data):
    data_dict = {}
    idx = data.find('\r\n\r\n') + 4
    post = data[idx:]
    pairs = post.split('&')
    for p in pairs:
        if '=' in p:
            key, val = p.split('=')
            data_dict[key] = val
    return data_dict


def web_page(m1_angle, m2_angle):
    html = f"""
    <html>
    <head><title>Stepper Control</title></head>
    <body style="font-family: Arial; text-align:center; margin-top:40px;">
        <h2>Stepper Motor Angle Control</h2>
        <form action="/" method="POST">
            <label>Motor 1 Angle (degrees):</label><br>
            <input type="text" name="m1" value="{m1_angle}"><br><br>

            <label>Motor 2 Angle (degrees):</label><br>
            <input type="text" name="m2" value="{m2_angle}"><br><br>

            <input type="submit" value="Rotate Motors"><br><br>
            
            <input type="submit" name="laser" value="Test Laser (3s)">
        </form>
    </body>
    </html>
    """
    return bytes(html, 'utf-8')


def serve_web(m1, m2):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('', 8080))
    s.listen(3)
    print("Web server running on port 8080...")

    while True:
        conn, addr = s.accept()
        msg = conn.recv(2048).decode()

        m1_target = ""
        m2_target = ""

        if msg.startswith("POST"):
            data = parsePOSTdata(msg)
        
            # Motor 1
            if "m1" in data and data["m1"].strip() != "":
                try:
                    m1_target = float(data["m1"])
                    p = m1.goAngle(m1_target)
                    p.join()
                except:
                    pass
        
            # Motor 2
            if "m2" in data and data["m2"].strip() != "":
                try:
                    m2_target = float(data["m2"])
                    p = m2.goAngle(m2_target)
                    p.join()
                except:
                    pass

            # Laser test button
            if "laser" in data:
                test_laser()

        response = web_page(m1.angle, m2.angle)
        conn.send(b'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n')
        conn.sendall(response)
        conn.close()

if __name__ == '__main__':
    s = Shifter(23, 24, 25)
    lock1 = multiprocessing.Lock()
    lock2 = multiprocessing.Lock()

    m1 = Stepper(s, lock1, 0)
    m2 = Stepper(s, lock2, 1)

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
