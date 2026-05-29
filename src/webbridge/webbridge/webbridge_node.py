import json
import math
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String as RosString

try:
    from vision_msgs.msg import Detection2DArray as _Detection2DArray
    _HAS_VISION_MSGS = True
except ImportError:
    _Detection2DArray = None
    _HAS_VISION_MSGS = False

PORT = 8081

# Module-level reference so the HTTP handler (instantiated per request) can
# reach the node without a closure.
_node_ref: "WebBridgeNode | None" = None


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        node = _node_ref
        if node is None:
            self.send_error(503)
            return
        p = self.path.split("?")[0]
        if p == "/api/camera":
            self._jpeg(node, "camera_jpeg")
        elif p == "/api/seg":
            self._jpeg(node, "seg_jpeg")
        elif p == "/api/lidar":
            self._json(node, "scan_json")
        elif p == "/api/odom":
            self._json(node, "odom_json")
        elif p == "/api/detections":
            self._json(node, "detections_json")
        elif p == "/api/buoys":
            self._json(node, "buoys_json")
        elif p == "/api/status":
            self._status(node)
        else:
            self.send_error(404)

    # ── helpers ──────────────────────────────────────────────────────────────

    def _jpeg(self, node, attr):
        with node._lock:
            data = getattr(node, attr)
        if data is None:
            self.send_response(204)
            self.end_headers()
            return
        self._respond(200, "image/jpeg", data)

    def _json(self, node, attr):
        with node._lock:
            data = getattr(node, attr)
        self._respond(200, "application/json", data or b"{}")

    def _status(self, node):
        with node._lock:
            body = json.dumps(
                {
                    "camera": node.camera_jpeg is not None,
                    "seg": node.seg_jpeg is not None,
                    "lidar": node.scan_json is not None,
                    "odom": node.odom_json is not None,
                }
            ).encode()
        self._respond(200, "application/json", body)

    def _respond(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", len(body))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)


class WebBridgeNode(Node):
    def __init__(self):
        super().__init__("webbridge_node")
        global _node_ref
        _node_ref = self

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self.camera_jpeg: bytes | None = None
        self.seg_jpeg: bytes | None = None
        self.scan_json: bytes | None = None
        self.odom_json: bytes | None = None
        self.detections_json: bytes | None = None
        self.buoys_json: bytes = b"[]"

        self.create_subscription(Image,     "/image_raw",         self._cb_camera, 1)
        self.create_subscription(Image,     "/yolo/seg_mask",     self._cb_seg,    1)
        self.create_subscription(LaserScan, "/scan",              self._cb_scan,   1)
        self.create_subscription(Odometry,  "/odometry/filtered", self._cb_odom,   1)
        self.create_subscription(RosString, "/buoys/nav",         self._cb_buoys,  10)
        if _HAS_VISION_MSGS:
            self.create_subscription(
                _Detection2DArray, "/yolo/detections", self._cb_detections, 1
            )
        else:
            self.get_logger().warn("vision_msgs not found — YOLO detections disabled")

        server = HTTPServer(("", PORT), _Handler)
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
        self.get_logger().info(f"WebBridge HTTP server on port {PORT}")

    # ── image helper ─────────────────────────────────────────────────────────

    def _to_jpeg(self, msg: Image, quality: int = 80) -> bytes | None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
            return bytes(buf) if ok else None
        except Exception as e:
            self.get_logger().warn(f"JPEG encode failed: {e}")
            return None

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _cb_camera(self, msg: Image):
        jpeg = self._to_jpeg(msg)
        with self._lock:
            self.camera_jpeg = jpeg

    def _cb_seg(self, msg: Image):
        jpeg = self._to_jpeg(msg)
        with self._lock:
            self.seg_jpeg = jpeg

    def _cb_scan(self, msg: LaserScan):
        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                pts.append([round(r * math.cos(angle), 3),
                             round(r * math.sin(angle), 3)])
            else:
                pts.append(None)
            angle += msg.angle_increment

        body = json.dumps(
            {
                "points": pts,
                "range_max": msg.range_max,
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
            }
        ).encode()
        with self._lock:
            self.scan_json = body

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = round(math.degrees(math.atan2(siny, cosy)), 1)

        v = msg.twist.twist.linear
        speed = round(math.sqrt(v.x**2 + v.y**2), 2)

        body = json.dumps(
            {
                "x": round(msg.pose.pose.position.x, 3),
                "y": round(msg.pose.pose.position.y, 3),
                "yaw_deg": yaw_deg,
                "speed": speed,
            }
        ).encode()
        with self._lock:
            self.odom_json = body

    def _cb_buoys(self, msg: RosString):
        with self._lock:
            self.buoys_json = msg.data.encode()

    def _cb_detections(self, msg: Detection2DArray):
        dets = []
        for det in msg.detections:
            b = det.bbox
            entry = {
                "cx": round(b.center.position.x, 1),
                "cy": round(b.center.position.y, 1),
                "w": round(b.size_x, 1),
                "h": round(b.size_y, 1),
            }
            if det.results:
                entry["class_id"] = det.results[0].hypothesis.class_id
                entry["score"] = round(det.results[0].hypothesis.score, 2)
            dets.append(entry)

        body = json.dumps({"detections": dets}).encode()
        with self._lock:
            self.detections_json = body


def main(args=None):
    rclpy.init(args=args)
    node = WebBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
