################################################################################
# WEB + STEPPER MOTORS + LASER CONTROL (WITH JSON POSITION LOADING)
################################################################################

import time
import socket
import threading
import requests
from shifter import Shifter
import RPi.GPIO as GPIO

print("Script starting...")

# --------------------------- CONFIG / CONSTANTS --------------------------------

GPIO.setmode(GPIO.BCM)
LASER_PIN = 17
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)

# JSON server and team #
JSON_URL = "http://192.168.1.254:8000/positions.json"
TEAM_ID = "1"

# Two 4-bit patterns, one per stepper
motor_patterns = [0, 0]


# ------------------------------ STEPPER CLASS ---------------------------------
class Stepper:
    seq = [0b0001, 0b0011, 0b0010, 0b0110,
           0b0100, 0b1100, 0b1000, 0b1001]  # half-step

    delay = 3000                    # us
    steps_per_degree = 2048 / 360.0  # 28BYJ-48 gear ratio 1:64

    def __init__(self, shifter, index):
        self.s = shifter
        self.index = index
        self.angle = 0.0
        self.step_state = 0

    def _normalize_angle(self, a):
        while a > 180: a -= 360
        while a < -180: a += 360
        return a

    def _step(self, direction):
        global motor_patterns

        self.step_state = (self.step_state + direction) % 8
        motor_patterns[self.index] = Stepper.seq[self.step_state]

        final = (motor_patterns[0] & 0x0F) | ((motor_patterns[1] & 0x0F) << 4)
        self.s.shiftByte(final)

        self.angle += direction / Stepper.steps_per_degree
        self.angle = self._normalize_angle(self.angle)

        time.sleep(Stepper.delay / 1e6)

    def rotate(self, delta):
        if delta == 0:
            return
        direction = 1 if delta > 0 else -1
        steps = int(abs(delta) * Stepper.steps_per_degree)
        for _ in range(steps):
            self._step(direction)

    def goAngle(self, target):
        target = max(-180, min(180, float(target)))
        curr = self.angle
        delta = target - curr

        if delta > 180: delta -= 360
        if delta < -180: delta += 360

        self.rotate(delta)

    def zero(self):
        self.angle = 0.0


# ----------------------------- LASER CONTROL ----------------------------------
def test_laser():
    GPIO.output(LASER_PIN, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(LASER_PIN, GPIO.LOW)


# ----------------------------- JSON LOADING ------------------------------------
def load_json_position():
    """
    Reads from JSON file and returns the (r, theta) for this turret.
    No rad/deg conversion—using theta directly.
    """
    try:
        print(f"Fetching JSON from {JSON_URL} ...")
        response = requests.get(JSON_URL, timeout=5)
        response.raise_for_status()

        data = response.json()

        # expected format: {"turrets":{"1":{"r":..., "theta":...}}, ...}
        turret = data["turrets"][TEAM_ID]

        r = turret["r"]
        theta = turret["theta"]  # radians — no conversion requested

        print(f"TURRET {TEAM_ID}: r={r}, theta={theta} (rad)")
        return theta  # We will treat theta (rad) as target angle

    except Exception as e:
        print("[ERROR] JSON load failed:", e)
        return 0.0


# ----------------------------- HTTP HELPERS -----------------------------------
def parsePOSTdata(data):
    d = {}
    idx = data.find('\r\n\r\n')
    if idx == -1:
        return d
    post = data[idx+4:]
    for p in post.split('&'):
        if '=' in p:
            k, v = p.split('=', 1)
            d[k] = v
    return d


# ------------------------------ WEB PAGE --------------------------------------
def web_page(m1_angle, m2_angle):
    return f"""
    <html>
    <head>
        <title>Turret Control</title>
        <style>
            body {{
                font-family: Arial;
                text-align: center;
                margin-top: 40px;
            }}
            .slider-container {{
                width: 60%;
                margin: 0 auto 30px auto;
            }}
        </style>
        <script>
            function send(body) {{
                var x = new XMLHttpRequest();
                x.open("POST", "/", true);
                x.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
                x.send(body);
            }}

            function setM1(v) {{
                document.getElementById("m1_val").innerHTML = v + " deg";
                send("m1=" + v);
            }}
            function setM2(v) {{
                document.getElementById("m2_val").innerHTML = v + " deg";
                send("m2=" + v);
            }}

            function laserTest() {{
                send("laser_test=1");
            }}

            function loadJSON() {{
                send("loadjson=1");
            }}
        </script>
    </head>

    <body>
        <h2>Stepper Motor + Laser Control</h2>

        <div class="slider-container">
            <h3>Motor 1 (Azimuth)</h3>
            <input type="range" min="-180" max="180" value="{m1_angle:.1f}"
                oninput="setM1(this.value)">
            <div>Angle: <span id="m1_val">{m1_angle:.1f} deg</span></div>
        </div>

        <div class="slider-container">
            <h3>Motor 2 (Altitude)</h3>
            <input type="range" min="-180" max="180" value="{m2_angle:.1f}"
                oninput="setM2(this.value)">
            <div>Angle: <span id="m2_val">{m2_angle:.1f} deg</span></div>
        </div>

        <button onclick="laserTest()">Test Laser</button>
        <button onclick="loadJSON()">Load JSON Position</button>
    </body>
    </html>
    """.encode("utf-8")


# ------------------------------ WEB SERVER ------------------------------------
def serve_web(m1, m2):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('', 8080))
    s.listen(3)
    print("Web server running on port 8080...")

    while True:
        conn, addr = s.accept()
        try:
            msg = conn.recv(4096).decode(errors='ignore')
            if not msg:
                conn.close()
                continue

            if msg.startswith("POST"):
                data = parsePOSTdata(msg)

                # Motor sliders
                if "m1" in data:
                    try:
                        m1.goAngle(float(data["m1"]))
                    except: pass

                if "m2" in data:
                    try:
                        m2.goAngle(float(data["m2"]))
                    except: pass

                # Laser
                if "laser_test" in data:
                    threading.Thread(target=test_laser, daemon=True).start()

                # JSON load → move motor 1
                if "loadjson" in data:
                    print("Loading JSON angle...")
                    theta = load_json_position()   # radians
                    deg = theta                   # Use raw value as angle
                    print(f"Moving to {deg} degrees (raw rad input)")
                    m1.goAngle(deg)

            # Send the HTML interface
            body = web_page(m1.angle, m2.angle)
            header = b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
            conn.sendall(header + body)

        finally:
            conn.close()


# ----------------------------------- MAIN -------------------------------------
def main():
    sh = Shifter(23, 24, 25)

    m1 = Stepper(sh, 0)
    m2 = Stepper(sh, 1)

    m1.zero()
    m2.zero()

    threading.Thread(target=serve_web, args=(m1, m2), daemon=True).start()

    print("Ready at:  http://<PI-IP>:8080")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
