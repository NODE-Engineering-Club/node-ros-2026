"""Fusion node — projects LIDAR point cloud onto camera segmentation mask.

For each LIDAR point:
  1. Transform from lidar frame → camera frame via TF
  2. Project into image plane using camera intrinsics
  3. Sample segmentation mask — if point lands on a detected object, it is
     confirmed as a labeled obstacle
  4. Points outside the camera FOV are passed through unconditionally

For each YOLO detection with NO LIDAR support (e.g. distant objects beyond
LIDAR range), a bearing estimate is added at DEFAULT_OBSTACLE_DISTANCE.

Output: /obstacles/fused (PointCloud2, frame: base_link)
"""

import math
import struct

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from vision_msgs.msg import Detection2DArray

# Camera intrinsic defaults — Logitech ~60° HFOV at 640x480
DEFAULT_FX = 554.0
DEFAULT_FY = 554.0
DEFAULT_CX = 320.0
DEFAULT_CY = 240.0
IMAGE_WIDTH  = 640
IMAGE_HEIGHT = 480

CAMERA_HFOV              = math.radians(60)
DEFAULT_OBSTACLE_DISTANCE = 5.0


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

        self.pub = self.create_publisher(PointCloud2, "/obstacles/fused", 10)
        self.create_subscription(PointCloud2,     "/obstacles/lidar",   self._lidar_cb, 10)
        self.create_subscription(Image,           "/yolo/seg_mask",     self._mask_cb,  _be_qos)
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
        """Return (tx, ty, tz) translation from lidar→camera frame, or None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._camera_frame, self._lidar_frame, rclpy.time.Time()
            )
            t = tf.transform.translation
            return t.x, t.y, t.z
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _project(self, x, y, z):
        """Project point (robot convention: x=fwd, y=left, z=up) to pixel (u, v).

        Returns (u, v) or None if point is behind camera.
        """
        if x <= 0:
            return None
        u = int(self._fx * (-y / x) + self._cx)
        v = int(self._fy * (-z / x) + self._cy)
        return u, v

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self):
        fused        = []
        det_has_lidar = set()  # indices of detections confirmed by LIDAR

        tf_offset = self._lidar_to_camera_transform()
        mask      = self._seg_mask
        h         = mask.shape[0] if mask is not None else IMAGE_HEIGHT
        w         = mask.shape[1] if mask is not None else IMAGE_WIDTH

        for (lx, ly, lz) in self._lidar_pts:
            fused.append((lx, ly, lz))  # always include LIDAR points

            # If TF and mask are available, correlate with detections
            if tf_offset is not None and mask is not None:
                tx, ty, tz = tf_offset
                cx, cy, cz = lx + tx, ly + ty, lz + tz
                uv = self._project(cx, cy, cz)
                if uv is not None:
                    u, v = uv
                    if 0 <= u < w and 0 <= v < h:
                        det_idx = int(mask[v, u])
                        if det_idx > 0:
                            det_has_lidar.add(det_idx - 1)  # mask value = det index + 1

        # Bearing estimate fallback for detections with no LIDAR coverage
        for i, det in enumerate(self._detections):
            if i not in det_has_lidar:
                bearing = (det.bbox.center.position.x / w - 0.5) * CAMERA_HFOV
                fused.append((
                    DEFAULT_OBSTACLE_DISTANCE * math.cos(bearing),
                    DEFAULT_OBSTACLE_DISTANCE * math.sin(bearing),
                    0.0,
                ))

        if not fused:
            return

        msg             = PointCloud2()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = "base_link"
        msg.height           = 1
        msg.width            = len(fused)
        msg.fields           = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = 12 * len(fused)
        msg.data         = b"".join(struct.pack("fff", *p) for p in fused)
        msg.is_dense     = True
        self.pub.publish(msg)


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
