"""MJPEG-over-HTTP streaming of the annotated frames.

The Orin is normally headless (JetPack's OpenCV is GTK2 and cannot open a
window over forwarded X11), so this is how you watch the live detection feed
from another machine on the LAN: point a browser at http://<orin-ip>:8080/.

Deliberately stdlib-only -- no Flask, no extra install on top of the JetPack
stack, and nothing to run on the viewing machine beyond a browser.
"""

import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

BOUNDARY = "frame"

_PAGE = b"""<!doctype html>
<html><head><title>RealSense D435i - live detections</title>
<style>
  body { margin:0; background:#111; color:#eee; font-family:system-ui,sans-serif;
         display:flex; flex-direction:column; align-items:center; gap:.75rem; padding:1rem; }
  img { max-width:100%; height:auto; image-rendering:pixelated; border-radius:4px; }
  p  { margin:0; font-size:.85rem; color:#999; }
</style></head>
<body>
  <img src="/stream.mjpg" alt="live detection feed">
  <p>RealSense D435i &mdash; YOLOv8 detections with XYZ in the color optical frame</p>
</body></html>
"""


class MJPEGServer:
    """Publishes the latest annotated frame to any number of browser clients.

    Frames are encoded only while at least one client is connected, so leaving
    the detection loop running headless with nobody watching costs nothing.
    """

    def __init__(self, port=8080, host="0.0.0.0", quality=80):
        self.port = port
        self.host = host
        self.quality = quality

        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self._clients = 0
        self._httpd = None
        self._thread = None

    @property
    def has_clients(self):
        return self._clients > 0

    def publish(self, image):
        """Encode and hand off a BGR frame. No-op when nobody is watching."""
        if self._clients == 0:
            return
        ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        with self._cond:
            self._jpeg = buf.tobytes()
            self._seq += 1
            self._cond.notify_all()

    def _wait_for_frame(self, last_seq, timeout=4.0):
        """Block until a frame newer than last_seq exists; (None, seq) on timeout."""
        with self._cond:
            if self._seq == last_seq:
                self._cond.wait(timeout)
            if self._seq == last_seq:
                return None, last_seq
            return self._jpeg, self._seq

    def start(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *args):
                pass  # keep the detection output readable

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._send_page()
                elif self.path.startswith("/stream.mjpg"):
                    self._send_stream()
                else:
                    self.send_error(404)

            def _send_page(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(_PAGE)))
                self.end_headers()
                self.wfile.write(_PAGE)

            def _send_stream(self):
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Connection", "close")
                self.send_header(
                    "Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}"
                )
                self.end_headers()

                server._clients += 1
                seq = -1
                try:
                    while True:
                        jpeg, seq = server._wait_for_frame(seq)
                        if jpeg is None:
                            continue  # detection loop stalled; hold the connection open
                        self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass  # viewer closed the tab
                finally:
                    server._clients -= 1

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None

    def url(self):
        """Best-guess LAN URL to hand to the user."""
        return f"http://{_lan_ip()}:{self.port}/"


def _lan_ip():
    """This host's address on the route out -- no traffic is actually sent."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))
        return s.getsockname()[0]
    except OSError:
        return socket.gethostbyname(socket.gethostname())
    finally:
        s.close()
