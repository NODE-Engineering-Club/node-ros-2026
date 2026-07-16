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
import tf2_geometry_msgs  # noqa: F401  (registers do_transform_point for PointStamped)
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
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

        self.declare_parameter("lidar_frame",       "lidar")
        self.declare_parameter("camera_frame",      "camera")
        self.declare_parameter("camera_info_topic", "/front_camera_driver/image_raw/camera_info")

        self._lidar_frame  = self.get_parameter("lidar_frame").get_parameter_value().string_value
        self._camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        camera_info_topic  = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        # Start with default estimates; overwritten on first camera_info message
        self._fx = DEFAULT_FX
        self._fy = DEFAULT_FY
        self._cx = DEFAULT_CX
        self._cy = DEFAULT_CY

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
        self.create_subscription(PointCloud2,      "/obstacles/lidar",  self._lidar_cb,     10)
        self.create_subscription(Image,            "/yolo/seg_mask",    self._mask_cb,      _be_qos)
        self.create_subscription(Detection2DArray, "/yolo/detections",  self._det_cb,       _be_qos)
        self.create_subscription(CameraInfo,       camera_info_topic,   self._camera_info_cb, 1)
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

    def _camera_info_cb(self, msg: CameraInfo):
        # K is the 3x3 row-major intrinsic matrix: [fx,0,cx, 0,fy,cy, 0,0,1]
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]

    # ── Projection ───────────────────────────────────────────────────────────

    def _lidar_to_camera_tf(self):
        """Return the full lidar->camera_frame TransformStamped, or None."""
        try:
            return self._tf_buffer.lookup_transform(
                self._camera_frame, self._lidar_frame, rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _project_calibrated(self, x, y, z, tf):
        """Project a lidar-frame point using the full calibrated TF
        (translation + rotation). front_camera_cal is guaranteed true
        optical convention (x-right, y-down, z-forward) by construction —
        it's built directly from solvePnP's rvec/tvec. Returns (u, v) or
        None if behind the camera.
        """
        ps = PointStamped()
        ps.point.x, ps.point.y, ps.point.z = float(x), float(y), float(z)
        p = tf2_geometry_msgs.do_transform_point(ps, tf).point
        if p.z <= 0:
            return None
        u = int(self._fx * p.x / p.z + self._cx)
        v = int(self._fy * p.y / p.z + self._cy)
        return u, v

    def _project_nominal(self, x, y, z, tf):
        """Legacy translation-only approximation for the uncalibrated
        (nominal front_camera) fallback frame, which isn't verified to
        match optical convention — kept unchanged from before the
        calibrated path existed. Returns (u, v) or None if behind camera.
        """
        t = tf.transform.translation
        cx, cy, cz = x + t.x, y + t.y, z + t.z
        if cx <= 0:
            return None
        u = int(self._fx * (-cy / cx) + self._cx)
        v = int(self._fy * (-cz / cx) + self._cy)
        return u, v

    def _project(self, x, y, z, tf):
        if self._camera_frame == "front_camera_cal":
            return self._project_calibrated(x, y, z, tf)
        return self._project_nominal(x, y, z, tf)

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self):
        fused        = []
        det_has_lidar = set()  # indices of detections confirmed by LIDAR

        tf   = self._lidar_to_camera_tf()
        mask = self._seg_mask
        h    = mask.shape[0] if mask is not None else IMAGE_HEIGHT
        w    = mask.shape[1] if mask is not None else IMAGE_WIDTH

        for (lx, ly, lz) in self._lidar_pts:
            fused.append((lx, ly, lz))  # always include LIDAR points

            # If TF and mask are available, correlate with detections
            if tf is not None and mask is not None:
                uv = self._project(lx, ly, lz, tf)
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
