################################################################################
# WEB + STEPPER MOTORS + LASER CONTROL
#  - Auto JSON load for team 17 placement
#  - Manual calibration: "Set as (0,0)" (center pole)
#  - Auto sequence: aim at each other turret and fire laser for 3s
################################################################################

import time
import socket
import threading
import json
import math
import urllib.request

from shifter import Shifter
import RPi.GPIO as GPIO

print("Script starting...")

# ----------------------- GPIO / LASER SETUP -----------------------------------

GPIO.setmode(GPIO.BCM)
LASER_PIN = 17
GPIO.setup(LASER_PIN, GPIO.OUT)
GPIO.output(LASER_PIN, GPIO.LOW)

# ----------------------- JSON CONFIG -----------------------------------------

JSON_URL = "http://192.168.1.254:8000/positions.json"
TEAM_ID = "17"          # our team number for placement coordinates

# Info to show on page
last_json_info = "No JSON loaded yet."

# cached positions (full JSON)
positions_data = None

# motor coil patterns (4 bits per motor)
motor_patterns = [0, 0]

# calibration offsets: physical angle that corresponds to logical 0°
calib_m1 = 0.0   # azimuth
calib_m2 = 0.0   # altitude

# auto sequence status
auto_status = "Idle"
auto_running = False


# ----------------------- STEPPER CLASS ---------------------------------------

class Stepper:
    # half-step sequence for 28BYJ-48
    seq = [0b0001, 0b0011, 0b0010, 0b0110,
           0b0100, 0b1100, 0b1000, 0b1001]

    delay = 3000                    # microseconds between steps
    steps_per_degree = 2048 / 360.0 # 28BYJ-48 gear ratio 1:64

    def __init__(self, shifter, index):
        self.s = shifter
        self.index = index    # 0 or 1
        self.angle = 0.0      # PHYSICAL angle in degrees, kept in [-180, 180]
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

        # store this motor's 4-bit pattern
        motor_patterns[self.index] = Stepper.seq[self.step_state]

        # pack both motors into one byte:
        # motor 0 → low nibble, motor 1 → high nibble
        final = (motor_patterns[0] & 0x0F) | ((motor_patterns[1] & 0x0F) << 4)
        self.s.shiftByte(final)

        # update physical angle
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
        # target is a PHYSICAL angle in degrees
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


# ----------------------- LASER CONTROL ---------------------------------------

def laser_on():
    GPIO.output(LASER_PIN, GPIO.HIGH)

def laser_off():
    GPIO.output(LASER_PIN, GPIO.LOW)

def test_laser():
    laser_on()
    time.sleep(3)
    laser_off()


# ----------------------- SIMPLE HELPERS --------------------------------------

def parsePOSTdata(data):
    d = {}
    idx = data.find("\r\n\r\n")
    if idx == -1:
        return d
    post = data[idx + 4:]
    for p in post.split("&"):
        if "=" in p:
            k, v = p.split("=", 1)
            d[k] = v
    return d


def normalize_angle(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


def normalize_rad(x):
    while x > math.pi:
        x -= 2 * math.pi
    while x <= -math.pi:
        x += 2 * math.pi
    return x


def logical_from_physical(physical, calib):
    # logical = physical - calib
    return normalize_angle(physical - calib)


def physical_from_logical(logical, calib):
    # physical target = calib + logical
    return normalize_angle(calib + logical)


# ----------------------- JSON LOADING / GEOMETRY -----------------------------

def fetch_positions():
    """
    Fetch full JSON and cache in positions_data.
    Also update last_json_info for our own team.
    """
    global positions_data, last_json_info
    try:
        print(f"Fetching JSON from {JSON_URL} ...")
        with urllib.request.urlopen(JSON_URL, timeout=40) as resp:
            raw = resp.read().decode("utf-8")
            positions_data = json.loads(raw)

        turret = positions_data["turrets"][TEAM_ID]
        r = float(turret["r"])
        theta = float(turret["theta"])  # radians

        last_json_info = (
            f"Team {TEAM_ID} placement (arena polar):\n"
            f"  r     = {r:.2f} cm\n"
            f"  theta = {theta:.6f} rad"
        )

        print("[OK] JSON loaded:")
        print(last_json_info)

    except Exception as e:
        positions_data = None
        last_json_info = f"Error loading JSON: {e}"
        print("[ERROR] JSON load failed:", e)


def compute_logical_azimuth_to_target(theta_self, theta_target):
    """
    Given our arena angle theta_self (rad) and target's arena angle theta_target (rad),
    compute the LOGICAL azimuth angle (deg) we must rotate to aim from our turret
    to the target, assuming logical 0° is pointing at the arena center.
    """
    # positions on unit circle (radius cancels)
    xs, ys = math.cos(theta_self), math.sin(theta_self)
    xt, yt = math.cos(theta_target), math.sin(theta_target)

    # vector from us to target
    vx, vy = xt - xs, yt - ys
    phi = math.atan2(vy, vx)      # global direction to target

    # global direction from us to center (0,0) is theta_self + pi
    phi_center = theta_self + math.pi

    d = normalize_rad(phi - phi_center)
    return math.degrees(d)


# ----------------------- AUTO SEQUENCE ---------------------------------------

def auto_sequence(m1, m2):
    """
    From current calibrated (0,0) position, aim at each other team's turret
    in order and fire laser for 3 seconds each.
    Altitude motor is not changed (assumed already calibrated to height).
    """
    global auto_status, auto_running, positions_data, calib_m1, calib_m2

    if positions_data is None:
        fetch_positions()
    if positions_data is None:
        auto_status = "Auto sequence aborted: no JSON data."
        return

    try:
        auto_running = True
        turrets = positions_data["turrets"]

        if TEAM_ID not in turrets:
            auto_status = f"Auto sequence aborted: TEAM_ID {TEAM_ID} missing."
            return

        theta_self = float(turrets[TEAM_ID]["theta"])

        # logical altitude remains 0 (center height)
        logical_alt = 0.0
        phys_alt = physical_from_logical(logical_alt, calib_m2)

        # move altitude motor to calibration height once
        m2.goAngle(phys_alt)

        # iterate through teams in numeric order, skipping ourselves
        for team_str, info in sorted(turrets.items(), key=lambda kv: int(kv[0])):
            if team_str == TEAM_ID:
                continue

            theta_target = float(info["theta"])
            logical_az = compute_logical_azimuth_to_target(theta_self, theta_target)
            auto_status = f"Aiming at Team {team_str} (azimuth {logical_az:.1f}°)"
            print(auto_status)

            phys_az = physical_from_logical(logical_az, calib_m1)
            m1.goAngle(phys_az)

            # fire laser for 3 seconds
            laser_on()
            time.sleep(3)
            laser_off()

            # small pause between shots
            time.sleep(0.5)

        auto_status = "Auto sequence finished."

    finally:
        laser_off()
        auto_running = False


# ----------------------- WEB PAGE --------------------------------------------

def web_page(log_m1_angle, log_m2_angle):
    """
    HTML UI with:
      - logical sliders (relative to calibrated (0,0))
      - laser test
      - Set as (0,0)
      - Start Auto Sequence
      - JSON info for team 17
      - Auto sequence status
    """
    global last_json_info, auto_status

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

        function setZero() {{
            send("set_zero=1");
        }}

        function startAuto() {{
            send("auto=1");
        }}
    </script>
</head>
<body>
    <h2>Stepper Motor + Laser Control</h2>
    <h3>Logical angles are relative to calibrated (0,0) position</h3>

    <div class="slider-container">
        <h3>Motor 1 (Azimuth)</h3>
        <input type="range" min="-180" max="180" value="{log_m1_angle:.1f}"
               oninput="setM1(this.value)">
        <div>Angle: <span id="m1_val">{log_m1_angle:.1f} deg</span></div>
    </div>

    <div class="slider-container">
        <h3>Motor 2 (Altitude)</h3>
        <input type="range" min="-180" max="180" value="{log_m2_angle:.1f}"
               oninput="setM2(this.value)">
        <div>Angle: <span id="m2_val">{log_m2_angle:.1f} deg</span></div>
    </div>

    <button onclick="laserTest()">Test Laser (3s)</button>
    <button onclick="setZero()">Set as (0,0)</button>
    <button onclick="startAuto()">Start Auto Sequence</button>

    <div class="box">
JSON Placement Data (Team {TEAM_ID})
------------------------------------
{last_json_info}
    </div>

    <div class="box">
Auto Sequence Status
--------------------
{auto_status}
    </div>
</body>
</html>
"""
    return html.encode("utf-8")


# ----------------------- WEB SERVER ------------------------------------------

def serve_web(m1, m2):
    global calib_m1, calib_m2, auto_running

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", 8080))
    s.listen(3)
    print("Web server running on port 8080...")

    try:
        while True:
            conn, addr = s.accept()
            try:
                msg = conn.recv(4096).decode("utf-8", errors="ignore")
                if not msg:
                    conn.close()
                    continue

                if msg.startswith("POST"):
                    data = parsePOSTdata(msg)

                    # slider angle for Motor 1 (logical)
                    if "m1" in data and data["m1"].strip() != "":
                        try:
                            log_target_m1 = float(data["m1"])
                            phys_target_m1 = physical_from_logical(log_target_m1, calib_m1)
                            m1.goAngle(phys_target_m1)
                        except ValueError:
                            pass

                    # slider angle for Motor 2 (logical)
                    if "m2" in data and data["m2"].strip() != "":
                        try:
                            log_target_m2 = float(data["m2"])
                            phys_target_m2 = physical_from_logical(log_target_m2, calib_m2)
                            m2.goAngle(phys_target_m2)
                        except ValueError:
                            pass

                    # laser test
                    if "laser_test" in data:
                        threading.Thread(target=test_laser, daemon=True).start()

                    # calibration: current physical angles become logical (0,0)
                    if "set_zero" in data:
                        calib_m1 = m1.angle
                        calib_m2 = m2.angle
                        print(f"[CALIBRATION] Set current position as (0,0): "
                              f"calib_m1={calib_m1:.2f}, calib_m2={calib_m2:.2f}")

                    # start auto sequence (if not already running)
                    if "auto" in data and not auto_running:
                        print("[AUTO] Starting auto sequence...")
                        threading.Thread(target=auto_sequence,
                                         args=(m1, m2),
                                         daemon=True).start()

                # compute logical angles for display (relative to calibration)
                log_m1 = logical_from_physical(m1.angle, calib_m1)
                log_m2 = logical_from_physical(m2.angle, calib_m2)

                # respond with current logical angles + JSON + status
                body = web_page(log_m1, log_m2)
                header = b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n"
                conn.sendall(header + body)

            finally:
                conn.close()
    finally:
        s.close()


# ----------------------- MAIN -------------------------------------------------

def main():
    global calib_m1, calib_m2

    sh = Shifter(23, 24, 25)

    m1 = Stepper(sh, 0)   # Motor 1 (azimuth)
    m2 = Stepper(sh, 1)   # Motor 2 (altitude)

    m1.zero()
    m2.zero()

    # initial calibration at this zero position
    calib_m1 = m1.angle
    calib_m2 = m2.angle

    # auto-load JSON placement data for team 17 on startup
    fetch_positions()

    # start web server thread
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


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        print("FATAL ERROR:")
        traceback.print_exc()
        GPIO.cleanup()
