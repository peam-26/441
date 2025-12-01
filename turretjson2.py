################################################################################
# WEB + STEPPER MOTORS + LASER CONTROL (+ JSON DISPLAY ONLY)
################################################################################

import time
import socket
import threading
import requests
from shifter import Shifter     # make sure shifter.py defines class Shifter
import RPi.GPIO as GPIO

print("Script starting...")

GPIO.setmode(GPIO.BCM)
LASER_PIN = 17
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)

# ---------- JSON CONFIG ----------
JSON_URL = "http://192.168.1.254:8000/positions.json"
TEAM_ID = "15"            # change if you really want team 1
last_json_info = "No JSON loaded yet."

motor_patterns = [0, 0]

class Stepper:
    seq = [0b0001, 0b0011, 0b0010, 0b0110,
           0b0100, 0b1100, 0b1000, 0b1001]

    delay = 3000                      # microseconds between steps
    steps_per_degree = 2048 / 360.0   # 28BYJ-48 typical

    def __init__(self, shifter, index):
        self.s = shifter
        self.index = index    # 0 or 1
        self.angle = 0.0      # current angle in degrees, kept in [-180, 180]
        self.step_state = 0   # index into seq[]

    def _normalize_angle(self, a):
        while a > 180:
            a -= 360
        while a < -180:
            a += 360
        return a

    def _step(self, direction):
        global motor_patterns

        self.step_state = (self.step_state + direction) % 8

        motor_patterns[self.index] = Stepper.seq[self.step_state]  # 0–15
        final = (motor_patterns[0] & 0x0F) | ((motor_patterns[1] & 0x0F) << 4)

        self.s.shiftByte(final)

        # Update angle
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
        target = max(-180.0, min(180.0, float(target)))
        current = self.angle
        delta = target - current
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        self.rotate(delta)

    def zero(self):
        self.angle = 0.0

def test_laser():
    GPIO.output(LASER_PIN, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(LASER_PIN, GPIO.LOW)

def parsePOSTdata(data):
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

# ---------- JSON LOADING (DISPLAY ONLY) ----------
def load_json_position():
    """
    Fetch JSON and update global last_json_info with r and theta for this team.
    No motor movement. No 'moving to ...' print.
    """
    global last_json_info
    try:
        print(f"Fetching JSON from {JSON_URL} ...")
        resp = requests.get(JSON_URL, timeout=40)   # longer timeout
        resp.raise_for_status()
        data = resp.json()

        turret = data["turrets"][TEAM_ID]
        r = float(turret["r"])
        theta = float(turret["theta"])  # radians

        last_json_info = (
            f"Team {TEAM_ID} turret coordinates:\n"
            f"  r     = {r:.2f} cm\n"
            f"  theta = {theta:.6f} rad"
        )

        print("[OK] JSON loaded:")
        print(last_json_info)

    except Exception as e:
        last_json_info = f"Error loading JSON: {e}"
        print("[ERROR] JSON load failed:", e)


def web_page(m1_angle, m2_angle):
    """
    HTML UI with JS sliders for live control, laser test,
    and JSON display box.
    """
    # use the global string we update when JSON is loaded
    global last_json_info

    html = f"""
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
            input[type=range] {{
                width: 100%;
            }}
            .box {{
                margin: 20px auto;
                padding: 15px;
                border: 1px solid #444;
                border-radius: 8px;
                width: 70%;
                text-align: left;
                background: #f9f9f9;
                white-space: pre-wrap;
                font-family: monospace;
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
                send("m1=" + encodeURIComponent(v));
            }}

            function setM2(v) {{
                document.getElementById("m2_val").innerHTML = v + " deg";
                send("m2=" + encodeURIComponent(v));
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

        <button onclick="laserTest()">Test Laser (3s)</button>
        <button onclick="loadJSON()">Load JSON Coordinates</button>

        <div class="box">
Last JSON Data
--------------
{last_json_info}
        </div>
    </body>
    </html>
    """
    return html.encode('utf-8')


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

                    # Slider angle for Motor 1
                    if "m1" in data and data["m1"].strip() != "":
                        try:
                            m1_target = float(data["m1"])
                            m1.goAngle(m1_target)
                        except ValueError:
                            pass

                    # Slider angle for Motor 2
                    if "m2" in data and data["m2"].strip() != "":
                        try:
                            m2_target = float(data["m2"])
                            m2.goAngle(m2_target)
                        except ValueError:
                            pass

                    # Laser test
                    if "laser_test" in data:
                        threading.Thread(target=test_laser, daemon=True).start()

                    # Load JSON (display only, no motor movement)
                    if "loadjson" in data:
                        load_json_position()

                # Always respond with current angles (page reload / initial load)
                response_body = web_page(m1.angle, m2.angle)
                header = b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n"
                conn.sendall(header + response_body)

            finally:
                conn.close()
    finally:
        s.close()

def main():
    # One shared shifter for both motors
    s = Shifter(23, 24, 25)

    m1 = Stepper(s, 0)   # Motor 1 (azimuth)
    m2 = Stepper(s, 1)   # Motor 2 (altitude)

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


if __name__ == '__main__':
    try:
        main()
    except Exception:
        import traceback
        print("FATAL ERROR:")
        traceback.print_exc()
        GPIO.cleanup()
