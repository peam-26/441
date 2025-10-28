import socket
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
LED_PINS = (12, 16, 20)
PWM_FREQ = 1000

for p in LED_PINS:
    GPIO.setup(p, GPIO.OUT)

pwms = [GPIO.PWM(p, PWM_FREQ) for p in LED_PINS]
for pwm in pwms:
    pwm.start(0)

levels = [0, 0, 0]

def set_level(idx, val):
    try:
        i = int(idx)
    except (ValueError, TypeError):
        return
    if i not in (0, 1, 2):
        return

    try:
        v = int(val)
    except (ValueError, TypeError):
        v = 0
    v = max(0, min(100, v))

    levels[i] = v
    pwms[i].ChangeDutyCycle(v)

def cleanup():
    for pwm in pwms:
        try:
            pwm.stop()
        except Exception:
            pass
    GPIO.cleanup()

def url_decode(s):
    out, i = [], 0
    while i < len(s):
        c = s[i]
        if c == '+':
            out.append(' '); i += 1
        elif c == '%' and i + 2 < len(s):
            try:
                out.append(bytes([int(s[i+1:i+3], 16)]).decode('utf-8', errors='ignore'))
                i += 3
            except ValueError:
                out.append('%'); i += 1
        else:
            out.append(c); i += 1
    return ''.join(out)

def parse_form_urlencoded(body_bytes):
    qs = body_bytes.decode('utf-8', errors='ignore')
    pairs = qs.split('&') if qs else []
    d = {}
    for pair in pairs:
        if not pair:
            continue
        k, v = (pair.split('=', 1) + [''])[:2]
        d[url_decode(k)] = url_decode(v)
    return d

def html_page(active_led=0, slider_val=None):
    l0, l1, l2 = levels[0], levels[1], levels[2]

    html = """\
<html>
<head>
  <meta charset="utf-8">
  <title>LED Brightness</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body style="font-family: Georgia, 'Times New Roman', Times, serif; margin: 1rem;">
  <div style="width: 380px; border: 2px solid #111; border-radius: 12px; padding: 12px 14px;">
    <!-- Row 1 -->
    <div style="display:flex; align-items:center; gap:12px; margin:10px 0;">
      <div style="width:64px;">LED1</div>
      <input id="s0" type="range" min="0" max="100" step="1" value="{l0}" style="flex:1;">
      <div id="v0" style="width:34px; text-align:right;">{l0}</div>
    </div>
    <!-- Row 2 -->
    <div style="display:flex; align-items:center; gap:12px; margin:10px 0;">
      <div style="width:64px;">LED2</div>
      <input id="s1" type="range" min="0" max="100" step="1" value="{l1}" style="flex:1;">
      <div id="v1" style="width:34px; text-align:right;">{l1}</div>
    </div>
    <!-- Row 3 -->
    <div style="display:flex; align-items:center; gap:12px; margin:10px 0;">
      <div style="width:64px;">LED3</div>
      <input id="s2" type="range" min="0" max="100" step="1" value="{l2}" style="flex:1;">
      <div id="v2" style="width:34px; text-align:right;">{l2}</div>
    </div>
  </div>

  <script>
    function postLevel(i, val) {{
      const body = "led=" + encodeURIComponent(i) + "&brightness=" + encodeURIComponent(val);
      fetch("/", {{
        method: "POST",
        headers: {{
          "Content-Type": "application/x-www-form-urlencoded",
          "X-Requested-With": "fetch"
        }},
        body
      }}).catch(() => {{}});
    }}

    for (let i = 0; i < 3; i++) {{
      const s = document.getElementById("s" + i);
      const v = document.getElementById("v" + i);
      s.addEventListener("input", () => {{
        v.textContent = s.value;   // live % on the right
        postLevel(i, s.value);     // send to server without reloading
      }});
    }}
  </script>
</body>
</html>
""".format(l0=l0, l1=l1, l2=l2)
    return html.encode("utf-8")


def parse_request(data):
    try:
        head, body = data.split(b"\r\n\r\n", 1)
    except ValueError:
        head, body = data, b""
    lines = head.split(b"\r\n")
    request_line = (lines[0] if lines else b"").decode("iso-8859-1")
    parts = request_line.split()
    method = parts[0] if len(parts) > 0 else ""
    path = parts[1] if len(parts) > 1 else "/"
    headers = {}
    for line in lines[1:]:
        if b":" in line:
            k, v = line.split(b":", 1)
            headers[k.decode("iso-8859-1").strip().lower()] = v.decode("iso-8859-1").strip()
    return method, path, headers, body

def read_full_request(conn):
    try:
        conn.settimeout(5.0)               
        data = conn.recv(4096)
    except TimeoutError:
        return b""                         
    except Exception:
        return b""
    if not data:
        return b""
    method, _, headers, body = parse_request(data)
    if method == "POST":
        try:
            cl = int(headers.get("content-length", "0") or "0")
        except ValueError:
            cl = 0
        have = len(body)
        while have < cl:
            try:
                chunk = conn.recv(4096)
            except TimeoutError:
                break
            if not chunk:
                break
            body += chunk
            have = len(body)
        head = data.split(b"\r\n\r\n", 1)[0]
        return head + b"\r\n\r\n" + body
    return data

def serve():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", 80))
    s.listen(3)
    try:
        while True:
            time.sleep(0.05)
            conn, _ = s.accept()
            try:
                raw = read_full_request(conn)
                if not raw:
                    try:
                        conn.sendall(b"HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n")
                    finally:
                        conn.close()
                    continue

                method, _, _, body = parse_request(raw)

                active_led, slider_val = 0, None
                if method == "POST":
                    form = parse_form_urlencoded(body)
                    idx = form.get("led", "0")
                    bright = form.get("brightness", "0")
                    set_level(idx, bright)
                    try:
                        active_led = int(idx)
                    except (ValueError, TypeError):
                        active_led = 0
                    try:
                        slider_val = int(bright)
                    except (ValueError, TypeError):
                        slider_val = None

                body_bytes = html_page(active_led, slider_val)
                headers_out = [
                    b"HTTP/1.1 200 OK",
                    b"Content-Type: text/html; charset=utf-8",
                    b"Connection: close",
                    f"Content-Length: {len(body_bytes)}".encode("ascii"),
                    b"\r\n",
                ]
                conn.sendall(b"\r\n".join(headers_out) + body_bytes)
            finally:
                conn.close()
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
        cleanup()

if __name__ == "__main__":
    serve()
