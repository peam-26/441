import socket

def generate_html():
    return """
    <html>
    <head><title>Test Page</title></head>
    <body>
        <h2>LED Control Test</h2>
        <form method="POST">
            <label>Brightness (0-100):</label><br>
            <input type="range" name="brightness" min="0" max="100" value="50"><br><br>
            <label>Choose LED:</label><br>
            <input type="radio" name="led" value="1" checked> LED 1<br>
            <input type="radio" name="led" value="2"> LED 2<br>
            <input type="radio" name="led" value="3"> LED 3<br><br>
            <input type="submit" value="Submit">
        </form>
    </body>
    </html>
    """

def start_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("0.0.0.0", 8080))
    s.listen(1)
    print("Go to http://<your-pi-ip>:8080 in a browser")
    while True:
        conn, addr = s.accept()
        request = conn.recv(1024).decode()
        print(request)  # just to see what comes in
        html = generate_html()
        response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" + html
        conn.sendall(response.encode())
        conn.close()

if __name__ == "__main__":
    start_server()
