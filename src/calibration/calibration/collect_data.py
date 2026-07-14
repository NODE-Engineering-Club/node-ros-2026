"""Interactive two-panel GUI for collecting LiDAR-camera point correspondences.

Method adapted from:
  [1] L. Zhang et al., "Calibration Method of 2D LIDAR and Camera Based on
      Indoor Structural Features," Hohai University.
      https://www.researching.cn/articles/OJbfdef44a334f8d3f
  [2] X. Zhong, camera_lidar_calibration (ROS1), GitHub, 2018.
      https://github.com/TurtleZhong/camera_lidar_calibration

Left panel  — undistorted camera image:  click to pick image pixel (u, v)
Right panel — top-down overhead LiDAR map: click to pick scan point (x, y)

Keyboard:
  f  freeze / unfreeze current frames
  a  add the selected pair to the collection
  s  save all pairs to ~/.ros/lidar_camera_data.txt
  c  run a quick inline calibration preview (requires ≥ 6 pairs)
  r  reset all collected pairs
  q  quit
"""

import math
import os
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers do_transform_point for PointStamped)
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

# Overhead map rendering
MAP_SIZE   = 600   # pixels
MAP_RANGE  = 5.0   # metres visible in each direction
MAP_CENTRE = MAP_SIZE // 2

# Fixed rotation from the URDF's front_camera frame's own local axes into true
# OpenCV optical convention (x=right, y=down, z=forward). Verified numerically
# against asket.urdf.xacro's front_camera_joint rpy — see calibrate_lidar_camera
# session notes. Only valid for the *nominal* (uncalibrated) front_camera frame;
# front_camera_cal (once calibrated) is already in optical convention directly.
_R_FIX = np.array([
    [0., -1., 0.],
    [1.,  0., 0.],
    [0.,  0., 1.],
])


def _lidar_to_map(x: float, y: float) -> tuple[int, int]:
    """Convert lidar (x forward, y left) to overhead map pixel."""
    px = int(MAP_CENTRE - y / MAP_RANGE * MAP_CENTRE)
    py = int(MAP_CENTRE - x / MAP_RANGE * MAP_CENTRE)
    return px, py


def _map_to_lidar(px: int, py: int) -> tuple[float, float]:
    """Convert overhead map pixel back to lidar (x, y) in metres."""
    y = -(px - MAP_CENTRE) / MAP_CENTRE * MAP_RANGE
    x = -(py - MAP_CENTRE) / MAP_CENTRE * MAP_RANGE
    return x, y


class CollectData(Node):
    def __init__(self):
        super().__init__("collect_data")

        self._bridge  = CvBridge()
        self._lock    = threading.Lock()
        self._frozen  = False

        # Latest live frames
        self._img_live:  np.ndarray | None = None
        self._scan_live: LaserScan | None  = None
        self._K:   np.ndarray | None = None
        self._D:   np.ndarray | None = None

        # Frozen frames (for clicking)
        self._img_frozen:  np.ndarray | None = None
        self._scan_frozen: LaserScan | None  = None

        # Pending selections
        self._sel_pixel: tuple[int, int] | None = None   # (u, v)
        self._sel_lidar: tuple[float, float] | None = None  # (x, y)

        # Collected pairs  [(x, y, z, u, v), ...]
        self._pairs: list[tuple] = []

        # Approximate (uncalibrated) LiDAR-scan-plane guide line, drawn on the
        # camera panel from the nominal URDF geometry — helps line up clicks
        # but is not the calibration result itself.
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._guide_pts: list[tuple[int, int]] | None = None

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image,      "/front_camera_driver/image_raw",              self._img_cb,  be_qos)
        self.create_subscription(CameraInfo, "/front_camera_driver/image_raw/camera_info",  self._info_cb, 1)
        self.create_subscription(LaserScan,  "/lidar_driver/scan_raw",                      self._scan_cb, be_qos)

        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
        cv2.namedWindow("LiDAR",  cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Camera", self._on_camera_click)
        cv2.setMouseCallback("LiDAR",  self._on_lidar_click)

        self.create_timer(1 / 20, self._render)

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _img_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self._lock:
            self._img_live = frame

    def _info_cb(self, msg: CameraInfo):
        with self._lock:
            self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._D = np.array(msg.d, dtype=np.float64)

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            self._scan_live = msg

    # ── Mouse callbacks ────────────────────────────────────────────────────────

    def _on_camera_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._sel_pixel = (x, y)
            self.get_logger().info(f"Image pixel selected: u={x}, v={y}")

    def _on_lidar_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            lx, ly = _map_to_lidar(x, y)
            self._sel_lidar = (lx, ly)
            self.get_logger().info(f"LiDAR point selected: x={lx:.3f} m, y={ly:.3f} m")

    # ── Guide line ─────────────────────────────────────────────────────────────

    def _compute_guide_line(self, K):
        """Project the LiDAR's z=0 scan plane onto the (undistorted) camera
        image using the nominal, uncalibrated base_link->lidar/front_camera
        TF from the URDF. Returns None if that TF isn't available yet (tried
        again next frame), else a list of (u, v) pixels — the visible sliver
        of a dense grid sampled across the plane, which is what a flat plane
        always projects to under perspective. This is an approximate guide
        for where to click, not a calibration result.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                "front_camera", "lidar", rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        h, w = 480, 640
        pts_optical = []
        for x in np.arange(0.2, 5.0, 0.1):
            for y in np.arange(-4.0, 4.0, 0.1):
                ps = PointStamped()
                ps.point.x, ps.point.y, ps.point.z = float(x), float(y), 0.0
                p_cam_local = tf2_geometry_msgs.do_transform_point(ps, tf).point
                # p_cam_local is in front_camera's own (non-optical) URDF axes;
                # rotate into true OpenCV optical convention (x-right, y-down,
                # z-forward) via the fixed correction verified against the URDF.
                v_local = np.array([p_cam_local.x, p_cam_local.y, p_cam_local.z])
                opt = _R_FIX @ v_local
                if opt[2] > 0.05:  # in front of the camera
                    pts_optical.append(opt)

        if not pts_optical:
            return None
        pts_optical = np.array(pts_optical, dtype=np.float64)
        # D=0: guide is drawn on the undistorted display (see _render).
        proj, _ = cv2.projectPoints(
            pts_optical, np.zeros(3), np.zeros(3), K, np.zeros(5)
        )
        proj = proj.reshape(-1, 2)
        pts = [(int(u), int(v)) for u, v in proj if 0 <= u < w and 0 <= v < h]
        return pts if pts else None

    # ── Rendering ──────────────────────────────────────────────────────────────

    def _render(self):
        with self._lock:
            img  = (self._img_frozen  if self._frozen else self._img_live)
            scan = (self._scan_frozen if self._frozen else self._scan_live)
            K, D = self._K, self._D

        if img is not None:
            display = img.copy()
            if K is not None and D is not None:
                display = cv2.undistort(display, K, D)
            if self._guide_pts is None and K is not None:
                self._guide_pts = self._compute_guide_line(K)
            if self._guide_pts:
                for u, v in self._guide_pts:
                    cv2.circle(display, (u, v), 1, (255, 0, 255), -1)
            if self._sel_pixel:
                cv2.drawMarker(display, self._sel_pixel, (0, 255, 0),
                               cv2.MARKER_CROSS, 20, 2)
            # Draw already-collected image points
            for _, _, _, u, v in self._pairs:
                cv2.circle(display, (int(u), int(v)), 5, (0, 200, 255), -1)
            label = "FROZEN" if self._frozen else "LIVE"
            cv2.putText(display, f"{label}  pairs:{len(self._pairs)}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            guide_note = "magenta = approx. LiDAR-height guide (uncalibrated)" if self._guide_pts else "guide: waiting for TF..."
            cv2.putText(display, guide_note, (10, display.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            cv2.imshow("Camera", display)

        if scan is not None:
            overhead = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
            # Grid
            for r in [1, 2, 3, 4]:
                pr = int(r / MAP_RANGE * MAP_CENTRE)
                cv2.circle(overhead, (MAP_CENTRE, MAP_CENTRE), pr, (40, 40, 40), 1)
            cv2.line(overhead, (0, MAP_CENTRE), (MAP_SIZE, MAP_CENTRE), (40, 40, 40), 1)
            cv2.line(overhead, (MAP_CENTRE, 0), (MAP_CENTRE, MAP_SIZE), (40, 40, 40), 1)
            # Robot origin
            cv2.drawMarker(overhead, (MAP_CENTRE, MAP_CENTRE), (0, 255, 255),
                           cv2.MARKER_STAR, 14, 2)

            angle = scan.angle_min
            for r in scan.ranges:
                if scan.range_min < r < scan.range_max:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    intensity = min(1.0, r / MAP_RANGE)
                    colour = (int(255 * (1 - intensity)), int(255 * intensity), 80)
                    cv2.circle(overhead, _lidar_to_map(x, y), 2, colour, -1)
                angle += scan.angle_increment

            if self._sel_lidar:
                px, py = _lidar_to_map(*self._sel_lidar)
                cv2.drawMarker(overhead, (px, py), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            for lx, ly, _, _, _ in self._pairs:
                cv2.circle(overhead, _lidar_to_map(lx, ly), 6, (0, 200, 255), -1)

            cv2.imshow("LiDAR", overhead)

        key = cv2.waitKey(1) & 0xFF
        self._handle_key(key)

    def _handle_key(self, key: int):
        if key == ord('f'):
            with self._lock:
                self._frozen = not self._frozen
                if self._frozen:
                    self._img_frozen  = self._img_live
                    self._scan_frozen = self._scan_live
            state = "frozen" if self._frozen else "live"
            self.get_logger().info(f"Frames {state}.")

        elif key == ord('a'):
            if self._sel_pixel is None:
                self.get_logger().warn("No image pixel selected — click on the Camera window first.")
                return
            if self._sel_lidar is None:
                self.get_logger().warn("No LiDAR point selected — click on the LiDAR window first.")
                return
            u, v = self._sel_pixel
            x, y = self._sel_lidar
            self._pairs.append((x, y, 0.0, float(u), float(v)))
            self.get_logger().info(
                f"Pair added #{len(self._pairs)}: lidar=({x:.3f},{y:.3f},0.0)  image=({u},{v})"
            )
            self._sel_pixel = None
            self._sel_lidar = None

        elif key == ord('s'):
            self._save()

        elif key == ord('c'):
            self._preview_calibration()

        elif key == ord('r'):
            self._pairs.clear()
            self._sel_pixel = None
            self._sel_lidar = None
            self.get_logger().info("Pairs reset.")

        elif key == ord('q'):
            rclpy.shutdown()

    def _save(self):
        if not self._pairs:
            self.get_logger().warn("No pairs to save.")
            return
        path = Path.home() / ".ros" / "lidar_camera_data.txt"
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as f:
            for x, y, z, u, v in self._pairs:
                f.write(f"{x:.6f} {y:.6f} {z:.6f} {u:.1f} {v:.1f}\n")
        self.get_logger().info(f"Saved {len(self._pairs)} pairs → {path}")

    def _preview_calibration(self):
        if len(self._pairs) < 6:
            self.get_logger().warn(f"Need ≥ 6 pairs (have {len(self._pairs)}).")
            return
        with self._lock:
            K, D = self._K, self._D
        if K is None:
            self.get_logger().warn("No camera_info received yet.")
            return
        pts3d = np.array([[x, y, z] for x, y, z, _, _ in self._pairs], dtype=np.float64)
        pts2d = np.array([[u, v]     for _, _, _, u, v in self._pairs], dtype=np.float64)
        # Points are clicked on the undistorted display (see _render), which
        # matches an ideal distortion-free pinhole camera — solve with D=0,
        # not the real D, or RANSAC silently fails to find a valid inlier set.
        D_solve = np.zeros_like(D)
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(pts3d, pts2d, K, D_solve)
        if not ok:
            self.get_logger().warn("solvePnPRansac failed.")
            return
        proj, _ = cv2.projectPoints(pts3d, rvec, tvec, K, D_solve)
        err = np.linalg.norm(pts2d - proj.squeeze(), axis=1).mean()
        self.get_logger().info(
            f"Preview: reprojection error = {err:.2f} px  "
            f"({inliers.shape[0] if inliers is not None else '?'}/{len(self._pairs)} inliers)  "
            f"t=[{tvec.flatten()}]"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CollectData()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
