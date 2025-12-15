################################################################################
# WEB + STEPPER MOTORS + LASER CONTROL
#  - Auto JSON load for team 17 (placement)
#  - Manual calibration via "Set as (0,0)" (center pole)
#  - Auto sequence:
#       1) Aim at each other turret & fire for 3s (azimuth only)
#       2) Return to center (0,0)
#       3) Aim at each globe & fire for 3s (azimuth + altitude)
#
#  Motor mapping (IMPORTANT):
#    - Motor index 0 -> ALTITUDE / tilt (laser up/down)
#    - Motor index 1 -> AZIMUTH / horizontal (spin)
################################################################################

import time
import socket
import threading
import requests
import math
from shifter import Shifter
import RPi.GPIO as GPIO

print("Script starting...")

# ---------- CONFIG ----------
JSON_URL = "http://192.168.1.254:8000/positions.json"
TEAM_ID = "17"            # our team number

LASER_PIN = 17

last_json_info = "No JSON loaded yet."
positions_data = None     # full JSON cache

auto_status = "Idle"
auto_running = False

motor_patterns = [0, 0]

# calibration (logical 0,0 at center pole)
# calib_alt = physical angle of altitude motor that corresponds to logical 0°
# calib_az  = physical angle of azimuth motor that corresponds to logical 0°
calib_alt = 0.0
calib_az  = 0.0

FIRE_TIME = 3.0  # seconds laser stays on per target


# ---------- STEPPER CLASS ----------
class Stepper:
    seq = [0b0001, 0b0011, 0b0010, 0b0110,
           0b0100, 0b1100, 0b1000, 0b1001]

    delay = 3000                      # microseconds between steps
    steps_per_degree = 2048 / 360.0   # 28BYJ-48 typical

    def __init__(self, shifter, index):
        self.s = shifter
        self.index = index    # 0 or 1
        self.angle = 0.0      # physical angle [-180, 180]
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
        # target = PHYSICAL angle
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


# ---------- LASER ----------
def laser_on():
    GPIO.output(LASER_PIN, GPIO.HIGH)

def laser_off():
    GPIO.output(LASER_PIN, GPIO.LOW)

def test_laser():
    laser_on()
    time.sleep(FIRE_TIME)
    laser_off()


# ---------- HELPERS ----------
def parsePOSTdata(data):
    d = {}
    idx = data.find('\r\n\r\n')
    if idx == -1:
        return d
    post = data[idx + 4:]
    for p in post.split('&'):
        if '=' in p:
            k, v = p.split('=', 1)
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


# ---------- JSON + GEOMETRY ----------
def load_positions_and_update_display():
    """
    Fetch JSON and:
      - store in positions_data (full JSON, including globes)
      - update last_json_info with our (r, theta)
    No globe math here – just loading and basic sanity.
    """
    global positions_data, last_json_info
    try:
        print(f"[JSON] Fetching {JSON_URL} ...")
        resp = requests.get(JSON_URL, timeout=40)
        print(f"[JSON] HTTP status: {resp.status_code}")
        resp.raise_for_status()

        data = resp.json()
        positions_data = data

        if "turrets" not in data:
            last_json_info = "Error: JSON has no 'turrets' key."
            print("[JSON] ERROR:", last_json_info)
            return

        if TEAM_ID not in data["turrets"]:
            last_json_info = f"Error: TEAM_ID {TEAM_ID} not in 'turrets'."
            print("[JSON] ERROR:", last_json_info)
            return

        turret = data["turrets"][TEAM_ID]
        r = float(turret["r"])
        theta = float(turret["theta"])

        last_json_info = (
            f"Team {TEAM_ID} turret placement (arena polar):\n"
            f"  r     = {r:.2f} cm\n"
            f"  theta = {theta:.6f} rad"
        )
        print("[JSON] OK:")
        print(last_json_info)

    except Exception as e:
        positions_data = None
        last_json_info = f"Error loading JSON (network/parse): {e!r}"
        print("[JSON] EXCEPTION:", repr(e))


def compute_logical_azimuth_to_target(theta_self, r_self, theta_target, r_target):
    """
    Compute logical azimuth angle (deg) needed to point from our turret
    to a target (turret or globe), assuming logical 0° = pointing at arena center.
    """
    xs, ys = r_self * math.cos(theta_self), r_self * math.sin(theta_self)
    xt, yt = r_target * math.cos(theta_target), r_target * math.sin(theta_target)

    vx, vy = xt - xs, yt - ys
    phi = math.atan2(vy, vx)      # global direction to target

    phi_center = theta_self + math.pi  # direction from us to center
    d = normalize_rad(phi - phi_center)
    return math.degrees(d)

def horizontal_distance(theta_self, r_self, theta_target, r_target):
    xs, ys = r_self * math.cos(theta_self), r_self * math.sin(theta_self)
    xt, yt = r_target * math.cos(theta_target), r_target * math.sin(theta_target)
    dx = xt - xs
    dy = yt - ys
    return math.sqrt(dx*dx + dy*dy)


# ---------- AUTO SEQUENCE ----------
def auto_sequence(m_alt, m_az):
    """
    From calibrated (0,0), run full sequence:
      1) Aim at each *other* turret and fire (azimuth only).
      2) Return to center (0,0).
      3) Aim at each globe (azimuth + altitude) and fire.
    """
    global auto_status, auto_running, positions_data, calib_alt, calib_az

    if positions_data is None:
        load_positions_and_update_display()
    if positions_data is None:
        auto_status = "Auto sequence aborted: no JSON data."
        print("[AUTO]", auto_status)
        return

    try:
        auto_running = True
        turrets = positions_data["turrets"]
        globes  = positions_data.get("globes", [])

        if TEAM_ID not in turrets:
            auto_status = f"Auto aborted: TEAM_ID {TEAM_ID} missing."
            print("[AUTO]", auto_status)
            return

        self_info = turrets[TEAM_ID]
        theta_self = float(self_info["theta"])
        r_self = float(self_info["r"])

        # ------------------ PHASE 1: OTHER TURRETS ------------------
        auto_status = "Starting turret sweep..."
        print("[AUTO]", auto_status)

        # altitude: keep logical 0 (whatever you calibrated for center)
        logical_alt = 0.0
        phys_alt_center = physical_from_logical(logical_alt, calib_alt)
        m_alt.goAngle(phys_alt_center)

        # sweep other turrets in numeric order
        for team_str, info in sorted(turrets.items(), key=lambda kv: int(kv[0])):
            if team_str == TEAM_ID:
                continue

            theta_target = float(info["theta"])
            r_target = float(info["r"])

            logical_az = compute_logical_azimuth_to_target(theta_self, r_self,
                                                           theta_target, r_target)
            auto_status = f"Aiming at Team {team_str} (az {logical_az:.1f}°)"
            print("[AUTO]", auto_status)

            phys_az = physical_from_logical(logical_az, calib_az)
            m_az.goAngle(phys_az)

            laser_on()
            time.sleep(FIRE_TIME)
            laser_off()
            time.sleep(0.5)

        # ------------------ PHASE 2: RETURN TO CENTER ------------------
        auto_status = "Returning to center (0,0)..."
        print("[AUTO]", auto_status)

        # logical (0,0) => physical calib_alt, calib_az
        m_alt.goAngle(physical_from_logical(0.0, calib_alt))
        m_az.goAngle(physical_from_logical(0.0, calib_az))

        # ------------------ PHASE 3: GLOBES ------------------
        if globes:
            auto_status = "Starting globe sweep..."
            print("[AUTO]", auto_status)

            for idx, g in enumerate(globes):
                r_g   = float(g["r"])
                th_g  = float(g["theta"])
                z_g   = float(g["z"])   # cm

                # horizontal geometry same as turrets
                logical_az = compute_logical_azimuth_to_target(theta_self, r_self,
                                                               th_g, r_g)

                # horizontal distance from us to globe
                d_horiz = horizontal_distance(theta_self, r_self, th_g, r_g)

                # altitude: turret at z=0, globe at z=z_g
                alt_rad = math.atan2(z_g, d_horiz) if d_horiz > 0 else (math.pi/2)
                logical_alt_globe = math.degrees(alt_rad)

                auto_status = (f"Aiming at Globe {idx+1} "
                               f"(az {logical_az:.1f}°, alt {logical_alt_globe:.1f}°)")
                print("[AUTO]", auto_status)

                # move altitude then azimuth
                phys_alt = physical_from_logical(logical_alt_globe, calib_alt)
                phys_az  = physical_from_logical(logical_az,          calib_az)

                m_alt.goAngle(phys_alt)
                m_az.goAngle(phys_az)

                laser_on()
                time.sleep(FIRE_TIME)
                laser_off()
                time.sleep(0.5)
        else:
            auto_status = "No globes in JSON; skipping globe phase."
            print("[AUTO]", auto_status)

        auto_status = "Auto sequence finished (turrets + globes)."
        print("[AUTO]", auto_status)

    finally:
        laser_off()
        auto_running = False


# ---------- WEB PAGE ----------
def web_page(log_alt_angle, log_az_angle):
    """
    - Motor 1 controls: ALTITUDE (tilt) via ±1° arrow buttons (logical)
    - Motor 2 controls: AZIMUTH (horizontal) via ±1° arrow buttons (logical)
    - Laser:
        * Test (3s)
        * ON (indefinite)
        * OFF
    """
    global last_json_info, auto_status

    alt_init = f"{log_alt_angle:.1f}"
    az_init  = f"{log_az_angle:.1f}"

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
            .arrow-btn {{
                width: 60px;
                height: 40px;
                font-size: 20px;
                margin: 5px;
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

            // current logical angles (deg), initialized from server
            var currAlt = {alt_init};
            var currAz  = {az_init};

            function clampAngle(a) {{
                if (a > 180) return 180;
                if (a < -180) return -180;
                return a;
            }}

            function updateLabels() {{
                document.getElementById("alt_val").innerHTML = currAlt.toFixed(1) + " deg";
                document.getElementById("az_val").innerHTML  = currAz.toFixed(1)  + " deg";
            }}

            function adjustAlt(delta) {{
                currAlt = clampAngle(currAlt + delta);
                updateLabels();
                send("alt=" + encodeURIComponent(currAlt));
            }}

            function adjustAz(delta) {{
                currAz = clampAngle(currAz + delta);
                updateLabels();
                send("az=" + encodeURIComponent(currAz));
            }}

            function laserTest() {{
                send("laser_test=1");
            }}
            function laserOnCmd() {{
                send("laser_on=1");
            }}
            function laserOffCmd() {{
                send("laser_off=1");
            }}

            function setZero() {{
                send("set_zero=1");
            }}

            function startAuto() {{
                send("auto=1");
            }}

            window.onload = function() {{
                updateLabels();
            }};
        </script>
    </head>
    <body>
        <h2>Stepper Motor + Laser Control</h2>
        <h3>Logical angles are relative to calibrated (0,0) position</h3>

        <div class="slider-container">
            <h3>Motor 1 (Altitude / Tilt)</h3>
            <button class="arrow-btn" onclick="adjustAlt(-1)">&larr;</button>
            <span id="alt_val">{log_alt_angle:.1f} deg</span>
            <button class="arrow-btn" onclick="adjustAlt(1)">&rarr;</button>
        </div>

        <div class="slider-container">
            <h3>Motor 2 (Azimuth / Horizontal)</h3>
            <button class="arrow-btn" onclick="adjustAz(-1)">&larr;</button>
            <span id="az_val">{log_az_angle:.1f} deg</span>
            <button class="arrow-btn" onclick="adjustAz(1)">&rarr;</button>
        </div>

        <div style="margin-top:20px;">
            <button onclick="laserTest()">Test Laser ({FIRE_TIME:.0f}s)</button>
            <button onclick="laserOnCmd()">Laser ON</button>
            <button onclick="laserOffCmd()">Laser OFF</button>
        </div>

        <div style="margin-top:20px;">
            <button onclick="setZero()">Set as (0,0)</button>
            <button onclick="startAuto()">Start Auto Sequence</button>
        </div>

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
    return html.encode('utf-8')


# ---------- WEB SERVER ----------
def serve_web(m_alt, m_az):
    global calib_alt, calib_az, auto_running

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

                    # Motor 1: ALTITUDE (logical)
                    if "alt" in data and data["alt"].strip() != "":
                        try:
                            log_target_alt = float(data["alt"])
                            phys_target_alt = physical_from_logical(log_target_alt, calib_alt)
                            m_alt.goAngle(phys_target_alt)
                        except ValueError:
                            pass

                    # Motor 2: AZIMUTH (logical)
                    if "az" in data and data["az"].strip() != "":
                        try:
                            log_target_az = float(data["az"])
                            phys_target_az = physical_from_logical(log_target_az, calib_az)
                            m_az.goAngle(phys_target_az)
                        except ValueError:
                            pass

                    # Laser controls
                    if "laser_test" in data:
                        threading.Thread(target=test_laser, daemon=True).start()
                    if "laser_on" in data:
                        laser_on()
                    if "laser_off" in data:
                        laser_off()

                    # Calibration: current physical angles become logical (0,0)
                    if "set_zero" in data:
                        calib_alt = m_alt.angle
                        calib_az  = m_az.angle
                        print(f"[CALIBRATION] Set current position as (0,0): "
                              f"calib_alt={calib_alt:.2f}, calib_az={calib_az:.2f}")

                    # Start auto sequence
                    if "auto" in data and not auto_running:
                        print("[AUTO] Starting auto sequence...")
                        threading.Thread(target=auto_sequence,
                                         args=(m_alt, m_az),
                                         daemon=True).start()

                # Logical angles for display
                log_alt = logical_from_physical(m_alt.angle, calib_alt)
                log_az  = logical_from_physical(m_az.angle,  calib_az)

                response_body = web_page(log_alt, log_az)
                header = b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n"
                conn.sendall(header + response_body)

            finally:
                conn.close()
    finally:
        s.close()


# ---------- MAIN ----------
def main():
    global calib_alt, calib_az

    # GPIO setup moved here so errors don't kill the script at import time
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(LASER_PIN, GPIO.OUT)
    GPIO.output(LASER_PIN, GPIO.LOW)

    sh = Shifter(23, 24, 25)

    # index 0 = ALTITUDE, index 1 = AZIMUTH
    m_alt = Stepper(sh, 0)
    m_az  = Stepper(sh, 1)

    m_alt.zero()
    m_az.zero()

    calib_alt = m_alt.angle
    calib_az  = m_az.angle

    # Try to load JSON once at startup (non-fatal if it fails)
    load_positions_and_update_display()

    t = threading.Thread(target=serve_web, args=(m_alt, m_az), daemon=True)
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
