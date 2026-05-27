import http.server
import json
import socketserver
import urllib.request
from pathlib import Path

PORT = 8080
MAVLINK_GPS = "http://localhost:6040/v1/mavlink/vehicles/1/components/1/messages/GLOBAL_POSITION_INT"
WEBBRIDGE   = "http://localhost:8081"


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/api/gps":
            self._proxy(MAVLINK_GPS)
        elif self.path.startswith("/api/"):
            self._proxy(WEBBRIDGE + self.path.split("?")[0])
        elif self.path in ("/", "/index.html"):
            self._serve("index.html", "text/html")
        else:
            self.send_response(404)
            self.end_headers()

    def _proxy(self, url):
        try:
            with urllib.request.urlopen(url, timeout=2) as r:
                body  = r.read()
                ctype = r.headers.get("Content-Type", "application/octet-stream")
                code  = r.status
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)
        except urllib.request.HTTPError as e:
            # Pass through 204 No Content transparently
            self.send_response(e.code)
            self.end_headers()
        except Exception as e:
            body = json.dumps({"error": str(e)}).encode()
            self.send_response(502)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)

    def _serve(self, filename, content_type):
        path = Path(__file__).parent / filename
        if path.exists():
            body = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()


print(f"Njord dashboard running on http://0.0.0.0:{PORT}")
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    httpd.serve_forever()
