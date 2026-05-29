"""Fusion node — projects LIDAR point cloud onto camera segmentation mask.

For each LIDAR point:
  1. Transform from lidar frame → camera frame via TF
  2. Project into image plane using camera intrinsics
  3. Sample segmentation mask — if point lands on a detected object, it is
     confirmed as a labeled obstacle and inherits the detection's color
  4. Points outside the camera FOV are passed through unconditionally

For each YOLO detection with NO LIDAR support (e.g. distant objects beyond
LIDAR range), a bearing estimate is added at DEFAULT_OBSTACLE_DISTANCE.

Output topics:
  /obstacles/fused  (PointCloud2, frame: base_link) — XYZ + color_id float
  /buoys/detected   (std_msgs/String, JSON)          — per-buoy color + position
"""

import json
import math
import struct

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from vision_msgs.msg import Detection2DArray

# Camera intrinsic defaults — Logitech ~60° HFOV at 640x480
DEFAULT_FX = 554.0
DEFAULT_FY = 554.0
DEFAULT_CX = 320.0
DEFAULT_CY = 240.0
IMAGE_WIDTH  = 640
IMAGE_HEIGHT = 480

CAMERA_HFOV               = math.radians(60)
DEFAULT_OBSTACLE_DISTANCE = 5.0

# Maps color name → float stored in PointCloud2 color_id field
COLOR_ID = {
    "unknown": 0.0, "red": 1.0, "green": 2.0, "yellow": 3.0,
    "black": 4.0, "white": 5.0, "blue": 6.0,
}


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")

        self.declare_parameter("fx",           DEFAULT_FX)
        self.declare_parameter("fy",           DEFAULT_FY)
        self.declare_parameter("cx",           DEFAULT_CX)
        self.declare_parameter("cy",           DEFAULT_CY)
        self.declare_parameter("lidar_frame",  "lidar")
        self.declare_parameter("camera_frame", "camera")

        self._fx           = self.get_parameter("fx").get_parameter_value().double_value
        self._fy           = self.get_parameter("fy").get_parameter_value().double_value
        self._cx           = self.get_parameter("cx").get_parameter_value().double_value
        self._cy           = self.get_parameter("cy").get_parameter_value().double_value
        self._lidar_frame  = self.get_parameter("lidar_frame").get_parameter_value().string_value
        self._camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._bridge      = CvBridge()

        self._lidar_pts: list[tuple[float, float, float]] = []
        self._seg_mask                                    = None
        self._detections                                  = []

        _be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.fused_pub = self.create_publisher(PointCloud2, "/obstacles/fused", 10)
        self.buoys_pub = self.create_publisher(String, "/buoys/detected", 10)
        self.create_subscription(PointCloud2,      "/obstacles/lidar",  self._lidar_cb, 10)
        self.create_subscription(Image,            "/yolo/seg_mask",    self._mask_cb,  _be_qos)
        self.create_subscription(Detection2DArray, "/yolo/detections",  self._det_cb,   _be_qos)
        self.create_timer(0.1, self._publish)  # 10 Hz

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _lidar_cb(self, msg):
        pts = []
        for i in range(msg.width):
            x, y, z = struct.unpack_from("fff", msg.data, i * msg.point_step)
            pts.append((x, y, z))
        self._lidar_pts = pts

    def _mask_cb(self, msg):
        self._seg_mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

    def _det_cb(self, msg):
        self._detections = msg.detections

    # ── Projection ───────────────────────────────────────────────────────────

    def _lidar_to_camera_transform(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._camera_frame, self._lidar_frame, rclpy.time.Time()
            )
            t = tf.transform.translation
            return t.x, t.y, t.z
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _project(self, x, y, z):
        if x <= 0:
            return None
        u = int(self._fx * (-y / x) + self._cx)
        v = int(self._fy * (-z / x) + self._cy)
        return u, v

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self):
        tf_offset = self._lidar_to_camera_transform()
        mask      = self._seg_mask
        h         = mask.shape[0] if mask is not None else IMAGE_HEIGHT
        w         = mask.shape[1] if mask is not None else IMAGE_WIDTH

        # Pass 1: map each LiDAR point to the detection index it lands on
        lidar_det: dict[int, int] = {}  # lidar_idx → det_idx
        det_has_lidar: set[int]  = set()

        if tf_offset is not None and mask is not None:
            tx, ty, tz = tf_offset
            for i, (lx, ly, lz) in enumerate(self._lidar_pts):
                uv = self._project(lx + tx, ly + ty, lz + tz)
                if uv is not None:
                    u, v = uv
                    if 0 <= u < w and 0 <= v < h:
                        det_idx = int(mask[v, u])
                        if det_idx > 0:
                            lidar_det[i] = det_idx - 1
                            det_has_lidar.add(det_idx - 1)

        # Pass 2: build fused point list with color_id (XYZC)
        fused: list[tuple[float, float, float, float]] = []
        det_positions: dict[int, list] = {}

        for i, (lx, ly, lz) in enumerate(self._lidar_pts):
            color_id = 0.0
            if i in lidar_det:
                d = lidar_det[i]
                color_id = self._det_color_id(d)
                det_positions.setdefault(d, []).append((lx, ly, lz))
            fused.append((lx, ly, lz, color_id))

        # Bearing-estimate fallback for detections with no LiDAR coverage
        for i, det in enumerate(self._detections):
            if i not in det_has_lidar:
                bearing = (det.bbox.center.position.x / w - 0.5) * CAMERA_HFOV
                bx = DEFAULT_OBSTACLE_DISTANCE * math.cos(bearing)
                by = DEFAULT_OBSTACLE_DISTANCE * math.sin(bearing)
                color_id = self._det_color_id(i)
                fused.append((bx, by, 0.0, color_id))
                det_positions.setdefault(i, [(bx, by, 0.0)])

        # Publish PointCloud2 (XYZ + color_id)
        if fused:
            self._pub_pointcloud(fused)

        # Publish /buoys/detected JSON
        self._pub_buoys(det_positions)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _det_color(self, det_idx: int) -> str:
        if det_idx < len(self._detections):
            det = self._detections[det_idx]
            if det.results:
                return det.results[0].hypothesis.class_id
        return "unknown"

    def _det_color_id(self, det_idx: int) -> float:
        return COLOR_ID.get(self._det_color(det_idx), 0.0)

    def _pub_pointcloud(self, fused):
        msg = PointCloud2()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.height          = 1
        msg.width           = len(fused)
        msg.fields          = [
            PointField(name="x",        offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",        offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",        offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="color_id", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step   = 16
        msg.row_step     = 16 * len(fused)
        msg.data         = b"".join(struct.pack("ffff", *p) for p in fused)
        msg.is_dense     = True
        self.fused_pub.publish(msg)

    def _pub_buoys(self, det_positions: dict):
        buoys = []
        for det_idx, pts in det_positions.items():
            color = self._det_color(det_idx)
            # Average position across all confirming LiDAR points
            mx = sum(p[0] for p in pts) / len(pts)
            my = sum(p[1] for p in pts) / len(pts)
            r  = math.sqrt(mx ** 2 + my ** 2)
            bearing_deg = math.degrees(math.atan2(my, mx))
            buoys.append({
                "color":       color,
                "x":           round(mx, 2),
                "y":           round(my, 2),
                "range":       round(r, 2),
                "bearing_deg": round(bearing_deg, 1),
            })
        msg = String()
        msg.data = json.dumps(buoys)
        self.buoys_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
