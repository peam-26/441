# led_webserver.py — socket-based POST form controlling 3 LED PWMs (no JS)

import socket
import urllib.parse
import RPi.GPIO as GPIO
import time
import atexit

GPIO.setmode(GPIO.BCM)
LED_PINS = (12, 16, 20)   # BCM pins for LED1..LED3
PWM_FREQ = 1000

for p in LED_PINS:
    GPIO.setup(p, GPIO.OUT)

pwms = [GPIO.PWM(p, PWM_FREQ) for p in LED_PINS]
for pwm in pwms:
    pwm.start(0)

levels = [0, 0, 0]  # brightness 0–100 for each LED

def set_level(idx: int, val: int):
    if idx not in (0, 1, 2):
        return
    v = max(0, min(100, int(val)))
    levels[idx] = v
    pwms[idx].ChangeDutyCycle(v)

@atexit.register
def _cleanup():
    for pwm in pwms:
        pwm.stop()
    GPIO.cleanup()

def html_page(active_led: int = 0, slider_val: int | None = None) -> bytes:
    if slider_val is None:
        slider_val = levels[active_led]
    chk0 = "checked" if active_led == 0 else ""
    chk1 = "checked" if active_led == 1 else ""
    chk2 = "checked" if active_led == 2 else ""

    html = """\
<html>
<head>
  <title>LED Brightness</title>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 2rem; max-width: 600px; }}
    fieldset {{ border: 1px solid #ccc; padding: 1rem; border-radius: .5rem; }}
    legend {{ font-weight: 700; }}
    .row {{ margin: .75rem 0; }}
    .levels {{ margin-top: 1.25rem; padding: 1rem; background:#f7f7f7; border-radius:.5rem; }}
    button {{ padding:.5rem 1rem; border-radius:.5rem; border:1px solid #333; background:#eee; cursor:pointer; }}
  </style>
</head>
<body>
  <h1>LED Brightness Controller</h1>
  <form method="POST" action="/">
    <fieldset>
      <legend>Select LED</legend>
      <div class="row">
        <label><input type="radio" name="led" value="0" {chk0}> LED 1</label><br>
        <label><input type="radio" name="led" value="1" {chk1}> LED 2</label><br>
        <label><input type="radio" name="led" value="2" {chk2}> LED 3</label>
      </div>
    </fieldset>

    <fieldset style="margin-top:1rem;">
      <legend>Brightness (0–100%)</legend>
      <div class="row">
        <input type="range" name="brightness" min="0" max="100" value="{slider}">
        <span>{slider}%</span>
      </div>
    </fieldset>

    <div class="row" style="margin-top:1rem;">
      <button type="submit">Set Brightness</button>
    </div>
  </form>

  <div class="levels">
    <strong>Current Levels</strong>
    <div>LED 1: {l0}%</div>
    <div>LED 2: {l1}%</div>
    <div>LED 3: {l2}%</div>
  </div>
</body>
</html>
""".format(
        chk0=chk0, chk1=chk1, chk2=chk2,
        slider=slider_val,
        l0=levels[0], l1=levels[1], l2=levels[2]
    )
    return html.encode("utf-8")

def parse_request(data: bytes):
    try:
        head, body = data.split(b"\r\n\r\n", 1)
    except ValueError:
        head, body = data, b""
    lines = head.split(b"\r\n")
    request_line = lines[0].decode("iso-8859-1")
    parts = request_line.split()
    method = parts[0] if len(parts) > 0 else ""
    path = parts[1] if len(parts) > 1 else "/"
    headers = {}
    for line in lines[1:]:
        if b":" in line:
            k, v = line.split(b":", 1)
            headers[k.decode().strip().lower()] = v.decode().strip()
    return method, path, headers, body

def read_full_request(conn):
    conn.settimeout(2.0)
    data = conn.recv(4096)
    if not data:
        return b""
    method, path, headers, body = parse_request(data)
    if method == "POST":
        cl = int(headers.get("content-length", "0") or "0")
        have = len(body)
        while have < cl:
            chunk = conn.recv(4096)
            if not chunk:
                break
            body += chunk
        head = data.split(b"\r\n\r\n", 1)[0]
        return head + b"\r\n\r\n" + body
    return data

def parse_form_urlencoded(body: bytes) -> dict:
    s = body.decode("utf-8", errors="ignore")
    return {k: (v[0] if isinstance(v, list) else v)
            for k, v in urllib.parse.parse_qs(s, keep_blank_values=True).items()}

def serve():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", 8080))   # use 80 if you want (requires sudo)
    s.listen(3)
    print("Serving on http://0.0.0.0:8080")
    try:
        while True:
            time.sleep(0.05)
            conn, _ = s.accept()
            try:
                raw = read_full_request(conn)
                if not raw:
                    conn.close(); continue
                method, path, headers, body = parse_request(raw)

                active_led = 0
                slider_val = None

                if method == "POST":
                    form = parse_form_urlencoded(body)
                    try:
                        idx = int(form.get("led", "0"))
                    except ValueError:
                        idx = 0
                    try:
                        bright = int(form.get("brightness", "0"))
                    except ValueError:
                        bright = 0
                    set_level(idx, bright)
                    active_led = idx
                    slider_val = bright

                body_bytes = html_page(active_led, slider_val)
                headers_out = [
                    b"HTTP/1.1 200 OK",
                    b"Content-Type: text/html; charset=utf-8",
                    b"Connection: close",
                    f"Content-Length: {len(body_bytes)}".encode(),
                    b"\r\n",
                ]
                conn.sendall(b"\r\n".join(headers_out) + body_bytes)
            finally:
                conn.close()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        s.close()

if __name__ == "__main__":
    serve()
