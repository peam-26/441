#throughout referenced lab 5, 7, 8; modules 6-8 for concurrency, network, motors
#referenced chatgpt for html help, some multithreading/concurrency, confirmed our geometry logic
import time
import socket
import threading
import requests
import math
from shifter import Shifter
import RPi.GPIO as GPIO

print("Starting")

#beginning setup
json_url = "http://192.168.1.254:8000/positions.json"
team_numb = "17"       
laser = 17

# used for webpage to keep visual update of when json is being parced, which team it will be firing at, with coordinates
# used this for debugging purposes
json_status = "No JSON loaded yet."
json_data = None   # store json file
auto_status = "Idle"
auto_running = False

motor_patterns = [0, 0]

# calibration to set 0,0 . altitude and azimuth
calib_alt = 0.0
calib_az  = 0.0

laser_fire = 3.0 # seconds

# Stepper class referenced from Lab 8
class Stepper:
    seq = [0b0001, 0b0011, 0b0010, 0b0110,
           0b0100, 0b1100, 0b1000, 0b1001] # CCW sequence

    delay = 3000 # delay between steps, microsec
    steps_per_degree = 4096 / 360.0 # 28BYJ-48, steps per degree of shaft rotation

    def __init__(self, shifter, motor):
        self.s = shifter
        self.motor = motor # 0 or 1 for the two motors
        self.angle = 0.0 # physical angle [-180, 180]
        self.step_state = 0   

    def _restrict_angle(self, a): # keep within -180, +180 degrees
        while a > 180:
            a -= 360
        while a < -180:
            a += 360
        return a

    def _step(self, direction):
        global motor_patterns
        self.step_state = (self.step_state + direction) % 8 # to know where it is in the CCW sequence

        motor_patterns[self.motor] = Stepper.seq[self.step_state]
        final = (motor_patterns[0] & 0x0F) | ((motor_patterns[1] & 0x0F) << 4)
        self.s.shiftByte(final) # send to shift register

        self.angle += direction / Stepper.steps_per_degree # update physical angle
        self.angle = self._restrict_angle(self.angle)

        time.sleep(Stepper.delay / 1e6)

    def rotate(self, delta): # Lab 8 reference
        if delta == 0:
            return
        direction = 1 if delta > 0 else -1 # +1 rotate CCW, -1 rotate CW
        steps = int(abs(delta) * Stepper.steps_per_degree) # find the right number of steps
        for _ in range(steps):
            self._step(direction)

    def goAngle(self, target): # move to angle, taken from Lab 8
        target = max(-180.0, min(180.0, float(target)))
        current = self.angle
        delta = target - current
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        self.rotate(delta)

    def zero(self): # reset to zero 
        self.angle = 0.0

#laser setup
def laser_on():
    GPIO.output(laser, GPIO.HIGH)

def laser_off():
    GPIO.output(laser, GPIO.LOW)

def test_laser():
    laser_on()
    time.sleep(laser_fire)
    laser_off()

# Referenced Lab 7, module 7, pg 20. helper function
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

def normalize_angle(a): # restrict angle -180, +180. Used for calibration
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a

def normalize_rad(x): # restrict angle -pi, +pi. Used for calibration
    while x > math.pi:
        x -= 2 * math.pi
    while x <= -math.pi:
        x += 2 * math.pi
    return x

# logic was explained by chatgpt, so that we understand how we can
# incorporate a calibration button
# physical angle is the real motor position, logical is defined by us when we set 0,0

def logical_from_physical(physical, calib): # to display angle
    return normalize_angle(physical - calib) # convert where motor actually is to where we think it should be

def physical_from_logical(logical, calib): # used later to move motor
    return normalize_angle(calib + logical) # converts desired angle to motor movement

#JSON conversion
def json_display():
    global json_data, json_status
    try:
        print(f"Reading {json_url} ...")
        resp = requests.get(json_url, timeout=40) # https://www.w3schools.com/python/ref_requests_get.asp, HTTP GET request reference
        print(f"HTTP status: {resp.status_code}") # debugging
        resp.raise_for_status() # error if HTTP error, go to except

        data = resp.json() # convert raw JSON
        json_data = data # store for automate

        if "turrets" not in data:
            json_status = "Error: JSON has no 'turrets' key."
            print("Error:", json_status)
            return

        if team_numb not in data["turrets"]:
            json_status = f"Error: team_numb {team_numb} not in 'turrets'."
            print("[Error:", json_status)
            return

        # get our position
        turret = data["turrets"][team_numb]
        r = float(turret["r"])
        theta = float(turret["theta"])

        json_status = (
            f"Team {team_numb} turret placement:\n"
            f"      r = {r:.2f} cm\n"
            f"  theta = {theta:.6f} rad"
        )
        print("Position grabbed:")
        print(json_status)

    except Exception as e:
        json_data = None
        json_status = f"Error loading JSON: {e}"

#bounced off ideas from chatgpt to get the right geometry from JSON
#convert coordinates to azimuth angle
def target_azimuth(theta_self, r_self, theta_target, r_target): # convert polar to cartesian
    xs, ys = r_self * math.cos(theta_self), r_self * math.sin(theta_self) # our self
    xt, yt = r_target * math.cos(theta_target), r_target * math.sin(theta_target) # target

    vx, vy = xt - xs, yt - ys # vector from turret to target
    phi = math.atan2(vy, vx) # global direction to target, - pi +pi

    #leveraged chatgpt to understand we needed: reference direction to center,
    #angle from origin to our turret and adding pi to flip direction
    phi_center = theta_self + math.pi  # direction from us to center
    d = normalize_rad(phi_center - phi) # relative angle from reference center to target
    return math.degrees(d)


def horizontal_distance(theta_self, r_self, theta_target, r_target):
    xs, ys = r_self * math.cos(theta_self), r_self * math.sin(theta_self)
    xt, yt = r_target * math.cos(theta_target), r_target * math.sin(theta_target)
    dx = xt - xs
    dy = yt - ys
    return math.sqrt(dx*dx + dy*dy)


#automation
def auto_sequence(m_alt, m_az):
    global auto_status, auto_running, json_data, calib_alt, calib_az

    if json_data is None:
        json_display()
    if json_data is None:
        auto_status = "Auto sequence error: no JSON data."
        print(auto_status)
        return

    try:
        auto_running = True
        turrets = json_data["turrets"]
        globes  = json_data.get("globes", [])

        if team_numb not in turrets:
            auto_status = f"Auto sequence error: team_numb {team_numb} missing."
            print(auto_status)
            return

        self_info = turrets[team_numb] # extract theta and distance for our turret
        theta_self = float(self_info["theta"])
        r_self = float(self_info["r"])

        #enemy targets sequence
        auto_status = "Auto sequence: Targeting enemy turrets"
        print(auto_status)

        #keep altitude at calibrated angle
        logical_alt = 0.0
        phys_alt_center = physical_from_logical(logical_alt, calib_alt)
        m_alt.goAngle(phys_alt_center)

        #target turrets one by one, referenced chatgpt to use "in sorted" to iterate turret numerical order 
        #turrets.keys to get all turret teams
        for team_str in sorted(turrets.keys(), key=int):
            info = turrets[team_str]

            if team_str == team_numb:
                continue

            theta_target = float(info["theta"])
            r_target = float(info["r"])

            logical_az = target_azimuth(theta_self, r_self, theta_target, r_target)
            auto_status = f"Aiming at Team {team_str} (az {logical_az:.1f}°)"
            print(auto_status)

            phys_az = physical_from_logical(logical_az, calib_az)
            m_az.goAngle(phys_az)

            laser_on()
            time.sleep(laser_fire)
            laser_off()
            time.sleep(0.5)

        #reset to 0,0 after enemy turrets
        auto_status = "Auto sequence: Returning to center (0,0)"
        print(auto_status)

        #with calibration, converts to motor angle for enemy targets
        m_alt.goAngle(physical_from_logical(0.0, calib_alt))
        m_az.goAngle(physical_from_logical(0.0, calib_az))

        #globe targeting sequence
        if globes:
            auto_status = "Auto sequence: Targeting globes"
            print(auto_status)

            #iterate and grab globe positions
            for idx, g in enumerate(globes):
                r_g = float(g["r"])
                th_g = float(g["theta"])
                z_g = float(g["z"])  

                #horizontal geometry same as turrets
                logical_az = target_azimuth(theta_self, r_self, th_g, r_g)
                d_horiz = horizontal_distance(theta_self, r_self, th_g, r_g)

                #leveraged chatgpt for altitude help
                #altitude: turret at z=0, globe at z=z_g
                alt_rad = math.atan2(z_g, d_horiz) if d_horiz > 0 else (math.pi/2)
                logical_alt_globe = math.degrees(alt_rad)

                auto_status = (f"Aiming at Globe {idx+1} "
                               f"(az {logical_az:.1f}°, alt {logical_alt_globe:.1f}°)")
                print(auto_status)

                # move altitude then azimuth
                phys_alt = physical_from_logical(logical_alt_globe, calib_alt)
                phys_az  = physical_from_logical(logical_az, calib_az)

                m_alt.goAngle(phys_alt)
                m_az.goAngle(phys_az)

                laser_on()
                time.sleep(laser_fire)
                laser_off()
                time.sleep(0.5)
        else: #debug
            auto_status = "Globes not found, skipping globe phase."
            print(auto_status)

        auto_status = "Auto sequence finished (turrets + globes)."
        print(auto_status)

    finally:
        laser_off()
        auto_running = False


#webpage,leveraged chatgpt for layout help
def web_page(log_alt_angle, log_az_angle):
    global json_status, auto_status

    alt_init = f"{log_alt_angle:.1f}"
    az_init  = f"{log_az_angle:.1f}"

#leveraged chatgpt to lay out html to create a webserver that allows us to manually calibrate motors, test laser, and start automation process
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
            <button onclick="laserTest()">Test Laser ({laser_fire:.0f}s)</button>
            <button onclick="laserOnCmd()">Laser ON</button>
            <button onclick="laserOffCmd()">Laser OFF</button>
        </div>

        <div style="margin-top:20px;">
            <button onclick="setZero()">Set as (0,0)</button>
            <button onclick="startAuto()">Start Auto Sequence</button>
        </div>

        <div class="box">
JSON Placement Data (Team {team_numb})
------------------------------------
{json_status}
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

#Reference Lab 7 and ddevoe-umd github for web_gpio_POST.py
#receive references to the altitude and azimuth motors
def serve_web(m_alt, m_az):
    global calib_alt, calib_az, auto_running

#create a TCP socket, also referenced Lab 7
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 8080))
    s.listen(3)
    print("Web server port 8080")

#blocks until a browser connects, mainly referenced module 7 but used chatgpt to help with clean up and threading
    try:
        while True:
            conn, addr = s.accept() #module 7 for sockets, page 7
            try:
                msg = conn.recv(4096).decode('utf-8', errors='ignore') #4096 bytes, HTTP request into text
                if not msg:
                    conn.close()
                    continue
                #referenced module 7, post requests
                if msg.startswith("POST"): 
                    data = parsePOSTdata(msg)

                    #motor 1 alt, logical to physical move motor angle
                    if "alt" in data and data["alt"].strip() != "":
                        try:
                            log_target_alt = float(data["alt"])
                            phys_target_alt = physical_from_logical(log_target_alt, calib_alt)
                            m_alt.goAngle(phys_target_alt)
                        except ValueError:
                            pass

                    #motor 2 azimuth, repeat same as above for motor angle
                    if "az" in data and data["az"].strip() != "":
                        try:
                            log_target_az = float(data["az"])
                            phys_target_az = physical_from_logical(log_target_az, calib_az)
                            m_az.goAngle(phys_target_az)
                        except ValueError:
                            pass

                    #control laser fire, concurrency/multithreading lecture
                    if "laser_test" in data:
                        threading.Thread(target=test_laser, daemon=True).start() #had issues where our web would freeze, threading fixed
                    if "laser_on" in data:
                        laser_on()
                    if "laser_off" in data:
                        laser_off()

                    #calibrate for 0,0 motor angles 
                    if "set_zero" in data:
                        calib_alt = m_alt.angle
                        calib_az  = m_az.angle
                        print(f"Set current position as (0,0): "f"calib_alt={calib_alt:.2f}, calib_az={calib_az:.2f}")

                    #start auto sequence, doesn't freeze up web 
                    if "auto" in data and not auto_running:
                        print("Start auto sequence")
                        threading.Thread(target=auto_sequence,args=(m_alt, m_az),daemon=True).start()

                #logical angles for display
                log_alt = logical_from_physical(m_alt.angle, calib_alt)
                log_az  = logical_from_physical(m_az.angle, calib_az)

                #used module 7 notes and chat for header 
                #to build HTML page and angles as well as JSON and auto sequence status
                response_body = web_page(log_alt, log_az)
                header = b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n\r\n"
                conn.sendall(header + response_body)

            finally:
                conn.close()
    finally:
        s.close()


#setup of the components of the turret and initializing the motors
def main():
    global calib_alt, calib_az

    #GPIO setup moved here so errors don't kill the script at import time
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(laser, GPIO.OUT)
    GPIO.output(laser, GPIO.LOW)

    sh = Shifter(23, 24, 25)

    #motor 0 = alt, motor 1 = azimuth
    m_alt = Stepper(sh, 0)
    m_az  = Stepper(sh, 1)

    m_alt.zero()
    m_az.zero()

    calib_alt = m_alt.angle
    calib_az  = m_az.angle

    #try to load JSON once at startup
    json_display()
    t = threading.Thread(target=serve_web, args=(m_alt, m_az), daemon=True)
    t.start()
    print("Motors and web are ready")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        GPIO.cleanup()

#refer lab 8 and ddevo enme441-pi github, stepper shift multiprocessing with chat suggestion for traceback
if __name__ == '__main__':
    try:
        main()
    except Exception:
        import traceback
        print("Major error:")
        traceback.print_exc()
        GPIO.cleanup()
