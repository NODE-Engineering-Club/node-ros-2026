"""Geo-referenced sensor fusion node.

This node wraps a lidar<->camera fusion pipeline that was written and validated
on real hardware as a standalone script. The tested core — Euclidean clustering,
bearing-based cluster<->detection association, and the constant-velocity Kalman
tracker — is preserved here verbatim. Only the I/O around it is different:

  * the standalone script read the camera (OpenCV) and RPLidar (serial) itself
    and ran YOLO inline; here those jobs belong to other nodes and arrive as
    topics, so the acquisition/inference/logging code is dropped;
  * the tracker runs in the boat (base_link) frame exactly as tested, and a
    final geo-referencing step (TF map<-base + the boat GPS fix) lifts each
    confirmed track into absolute lat/lon so the published message is unchanged.

  inputs:
    /yolo/detections    vision_msgs/Detection2DArray   (class + bbox)
    /obstacles/lidar    sensor_msgs/PointCloud2         (metric points, lidar frame)
    /gps_driver/gps_raw sensor_msgs/NavSatFix           (boat absolute lat/lon)

  output:
    /obstacles/global   njord_msgs/ObstacleArray        (frame: map)

Calibration: lidar_yaw_offset_deg / offsets / camera_hfov_deg below are the
extrinsic knobs from the tested rig. They must be re-tuned for the boat's actual
camera<->lidar mounting (see the standalone script's notes).

Behaviour note: like the tested tracker, only lidar-confirmed objects are
tracked and published. Camera-only (unranged) detections are associated but not
emitted as obstacles.
"""

import math
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, PointCloud2
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from vision_msgs.msg import Detection2DArray

from geographic_msgs.msg import GeoPoint
from njord_msgs.msg import Obstacle, ObstacleArray

import tf2_geometry_msgs  # noqa: F401  (registers do_transform_point for PointStamped)

WGS84_A = 6378137.0  # earth equatorial radius (m), good enough for local tangent plane


class GeoFusionNode(Node):
    def __init__(self):
        super().__init__("geo_fusion_node")

        # ── Parameters (defaults = values validated on the test rig) ──────────
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_hfov_deg", 60.0)
        self.declare_parameter("image_width", 640)
        # extrinsic calibration (lidar -> camera/base), the main tuning knobs
        self.declare_parameter("lidar_yaw_offset_deg", 0.0)
        self.declare_parameter("lidar_offset_x_m", 0.0)
        self.declare_parameter("lidar_offset_y_m", 0.0)
        # clustering / association
        self.declare_parameter("cluster_tolerance", 0.5)
        self.declare_parameter("min_cluster_points", 2)
        self.declare_parameter("bearing_match_tol_deg", 8.0)
        self.declare_parameter("confidence_min", 0.5)
        # sync
        self.declare_parameter("sync_slop", 0.15)
        self.declare_parameter("sync_queue", 10)
        # Kalman tracker
        self.declare_parameter("track_gate_m", 1.5)
        self.declare_parameter("track_process_var", 1.0)
        self.declare_parameter("track_meas_var", 0.10)
        self.declare_parameter("track_confirm_hits", 3)
        self.declare_parameter("track_max_misses", 8)

        p = self.get_parameter
        self._base_frame = p("base_frame").value
        self._map_frame = p("map_frame").value
        self._hfov = math.radians(p("camera_hfov_deg").value)
        self._img_w = p("image_width").value
        theta = math.radians(p("lidar_yaw_offset_deg").value)
        self._cos_t, self._sin_t = math.cos(theta), math.sin(theta)
        self._off_x = p("lidar_offset_x_m").value
        self._off_y = p("lidar_offset_y_m").value
        self._cluster_tol = p("cluster_tolerance").value
        self._min_cluster = p("min_cluster_points").value
        self._bearing_tol = math.radians(p("bearing_match_tol_deg").value)
        self._conf_min = p("confidence_min").value

        self._tracker = Tracker(
            gate_m=p("track_gate_m").value,
            process_var=p("track_process_var").value,
            meas_var=p("track_meas_var").value,
            confirm_hits=p("track_confirm_hits").value,
            max_misses=p("track_max_misses").value,
        )
        self._t_prev = None  # ros Time of previous fused frame, for tracker dt

        # ── TF ────────────────────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Boat GPS fix (latest) ───────────────────────────────────────────────
        self._fix = None
        self.create_subscription(NavSatFix, "/gps_driver/gps_raw", self._fix_cb, 10)

        # ── Synchronised vision + lidar ─────────────────────────────────────────
        # /yolo/detections is BEST_EFFORT (see vision/node.py); lidar is RELIABLE.
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        det_sub = Subscriber(self, Detection2DArray, "/yolo/detections", qos_profile=be_qos)
        lidar_sub = Subscriber(self, PointCloud2, "/obstacles/lidar")
        self._sync = ApproximateTimeSynchronizer(
            [det_sub, lidar_sub],
            queue_size=p("sync_queue").value,
            slop=p("sync_slop").value,
        )
        self._sync.registerCallback(self._fused_cb)

        self.pub = self.create_publisher(ObstacleArray, "/obstacles/global", 10)

        self.get_logger().info("geo_fusion_node ready — publishing /obstacles/global")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _fix_cb(self, msg):
        self._fix = msg

    def _fused_cb(self, det_msg, lidar_msg):
        if self._fix is None:
            self.get_logger().warn("No GPS fix yet — cannot geo-reference", throttle_duration_sec=5.0)
            return

        stamp = lidar_msg.header.stamp

        # Only TF we need: map <- base (for heading + lifting tracks to lat/lon).
        # lidar -> base is handled by the extrinsic calibration in lidar_to_camera.
        t_map_base = self._lookup(self._map_frame, self._base_frame, stamp)
        if t_map_base is None:
            self.get_logger().warn("TF map<-base unavailable — skipping frame", throttle_duration_sec=5.0)
            return
        boat_map = t_map_base.transform.translation
        boat_heading = _yaw_from_quat(t_map_base.transform.rotation)

        # Parse ROS detections into the form fuse() expects. Upstream already
        # thresholds/labels; we keep a confidence gate, convert the bbox centre
        # pixel to a bearing, and pass the string class through (the tested
        # script used int ids + a name table).
        dets = []
        for d in det_msg.detections:
            if not d.results:
                continue
            hyp = max(d.results, key=lambda r: r.hypothesis.score)
            if hyp.hypothesis.score < self._conf_min:
                continue
            dets.append({
                "bearing": self._bbox_bearing(d.bbox.center.position.x),
                "class": hyp.hypothesis.class_id,  # string label
                "conf": hyp.hypothesis.score,
                "matched": False,
            })

        lidar_points = _read_xy(lidar_msg)
        objects = self.fuse(dets, lidar_points)

        # tracker dt from the node clock (mirrors the tested script's wall time).
        now = self.get_clock().now()
        dt = 1e-3 if self._t_prev is None else (now - self._t_prev).nanoseconds * 1e-9
        self._t_prev = now
        tracks = self._tracker.update(objects, dt)

        # Assemble + publish. Each confirmed track lives in the base frame; lift
        # it to map (TF) then to absolute lat/lon (boat GPS as tangent anchor).
        out = ObstacleArray()
        out.header.stamp = stamp
        out.header.frame_id = self._map_frame
        out.boat_position = GeoPoint(
            latitude=self._fix.latitude,
            longitude=self._fix.longitude,
            altitude=self._fix.altitude,
        )
        out.boat_heading = boat_heading
        out.obstacles = [self._make_obstacle(tr, t_map_base, boat_map) for tr in tracks]
        self.pub.publish(out)

    # ── Fusion (ported from the tested standalone script) ──────────────────────

    def lidar_to_camera(self, xl, yl):
        xc = self._cos_t * xl - self._sin_t * yl + self._off_x
        yc = self._sin_t * xl + self._cos_t * yl + self._off_y
        return xc, yc

    def fuse(self, dets, lidar_points):
        objects = []

        cam_points = [self.lidar_to_camera(x, y) for (x, y) in lidar_points]
        clusters = _euclidean_cluster(cam_points, self._cluster_tol, self._min_cluster)
        for cluster in clusters:
            cx = sum(px for px, _ in cluster) / len(cluster)
            cy = sum(py for _, py in cluster) / len(cluster)
            rng = math.hypot(cx, cy)
            bearing = math.atan2(cy, cx)
            radius = max(math.hypot(px - cx, py - cy) for px, py in cluster)

            best, best_err = None, self._bearing_tol
            for d in dets:
                if d["matched"]:
                    continue
                err = abs(_wrap(d["bearing"] - bearing))
                if err < best_err:
                    best, best_err = d, err
            if best is not None:
                best["matched"] = True
                cls, conf = best["class"], best["conf"]
            else:
                cls, conf = "unknown", 0.0

            objects.append({
                "class": cls if cls is not None else "unknown",
                "confidence": conf,
                "range_m": rng,
                "bearing_deg": math.degrees(bearing),
                "radius": radius,
                "lidar_confirmed": True,
            })

        for d in dets:
            if d["matched"]:
                continue
            objects.append({
                "class": d["class"],
                "confidence": d["conf"],
                "range_m": float("nan"),
                "bearing_deg": math.degrees(d["bearing"]),
                "radius": 0.0,
                "lidar_confirmed": False,
            })

        return objects

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _bbox_bearing(self, u):
        """Bbox centre pixel -> bearing rel. camera forward (+x). Left of image
        (small u) -> boat's port side (+y) -> positive bearing."""
        return (0.5 - u / self._img_w) * self._hfov

    def _lookup(self, target, source, stamp):
        """Full transform target<-source at stamp, or latest, or None."""
        for t in (stamp, rclpy.time.Time()):
            try:
                return self._tf_buffer.lookup_transform(target, source, t)
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue
        return None

    @staticmethod
    def _apply(transform, x, y):
        """Apply a TransformStamped to a 2D point (z=0). Returns geometry_msgs/Point."""
        ps = PointStamped()
        ps.point.x = float(x)
        ps.point.y = float(y)
        ps.point.z = 0.0
        return tf2_geometry_msgs.do_transform_point(ps, transform).point

    def _make_obstacle(self, track, t_map_base, boat_map):
        x, y = track.pos                      # base frame: x forward, y port/left
        in_map = self._apply(t_map_base, x, y)

        obs = Obstacle()
        obs.id = int(track.id) + 1             # +1: msg reserves id 0 for "not tracked"
        obs.class_id = track.cls
        obs.confidence = float(track.conf)
        obs.position = self._to_geo(in_map.x, in_map.y, boat_map)
        obs.position_map = Point(x=in_map.x, y=in_map.y, z=in_map.z)
        obs.radius = float(track.radius)      # cluster spread (m), from the lidar points
        obs.lidar_confirmed = True            # only lidar-confirmed tracks are reported
        obs.range_m = float(math.hypot(x, y))
        obs.bearing_deg = float(math.degrees(math.atan2(y, x)))
        obs.speed_mps = float(track.speed)
        return obs

    def _to_geo(self, mx, my, boat_map):
        """Convert a map-frame point (ENU metres) to absolute lat/lon using the
        boat's current GPS fix as the local-tangent anchor."""
        lat0 = self._fix.latitude
        lon0 = self._fix.longitude
        dn = my - boat_map.y  # metres north of boat
        de = mx - boat_map.x  # metres east of boat
        lat = lat0 + math.degrees(dn / WGS84_A)
        lon = lon0 + math.degrees(de / (WGS84_A * math.cos(math.radians(lat0))))
        return GeoPoint(latitude=lat, longitude=lon, altitude=self._fix.altitude)


# ── Kalman tracker (ported verbatim from the tested standalone script) ─────────

class _Track:
    _next_id = 0

    def __init__(self, x, y, cls, conf, radius=0.0):
        self.id = _Track._next_id
        _Track._next_id += 1
        self.X = np.array([x, y, 0.0, 0.0])
        self.P = np.diag([0.5, 0.5, 4.0, 4.0])
        self.cls = cls
        self.conf = conf
        self.radius = radius
        self.hits = 1
        self.misses = 0
        self.confirmed = False

    @property
    def pos(self):
        return self.X[0], self.X[1]

    @property
    def speed(self):
        return math.hypot(self.X[2], self.X[3])


_H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])


class Tracker:
    def __init__(self, gate_m, process_var, meas_var, confirm_hits, max_misses):
        self.tracks = []
        self.gate2 = gate_m * gate_m
        self.q = process_var
        self.R = np.eye(2) * meas_var
        self.confirm_hits = confirm_hits
        self.max_misses = max_misses

    def _predict(self, tr, dt):
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        q = self.q
        Q = q * np.array([[dt4 / 4, 0, dt3 / 2, 0],
                          [0, dt4 / 4, 0, dt3 / 2],
                          [dt3 / 2, 0, dt2, 0],
                          [0, dt3 / 2, 0, dt2]])
        tr.X = F @ tr.X
        tr.P = F @ tr.P @ F.T + Q

    def _update(self, tr, z):
        S = _H @ tr.P @ _H.T + self.R
        K = tr.P @ _H.T @ np.linalg.inv(S)
        tr.X = tr.X + K @ (z - _H @ tr.X)
        tr.P = (np.eye(4) - K @ _H) @ tr.P
        tr.hits += 1
        tr.misses = 0
        if tr.hits >= self.confirm_hits:
            tr.confirmed = True

    def update(self, objects, dt):
        if dt <= 0:
            dt = 1e-3

        for tr in self.tracks:
            self._predict(tr, dt)

        meas = []
        for o in objects:
            if o["lidar_confirmed"] and not math.isnan(o["range_m"]):
                b = math.radians(o["bearing_deg"])
                meas.append((o["range_m"] * math.cos(b),
                             o["range_m"] * math.sin(b), o))

        pairs = []
        for mi, (mx, my, _) in enumerate(meas):
            for ti, tr in enumerate(self.tracks):
                dx, dy = mx - tr.X[0], my - tr.X[1]
                d2 = dx * dx + dy * dy
                if d2 <= self.gate2:
                    pairs.append((d2, mi, ti))
        pairs.sort()
        used_m, used_t = set(), set()
        for d2, mi, ti in pairs:
            if mi in used_m or ti in used_t:
                continue
            used_m.add(mi)
            used_t.add(ti)
            mx, my, o = meas[mi]
            tr = self.tracks[ti]
            self._update(tr, np.array([mx, my]))
            tr.cls, tr.conf = o["class"], o["confidence"]
            tr.radius = o["radius"]
            o["track_id"] = tr.id
            o["speed_mps"] = tr.speed

        for ti, tr in enumerate(self.tracks):
            if ti not in used_t:
                tr.misses += 1
        for mi, (mx, my, o) in enumerate(meas):
            if mi not in used_m:
                tr = _Track(mx, my, o["class"], o["confidence"], o["radius"])
                self.tracks.append(tr)
                o["track_id"] = tr.id
                o["speed_mps"] = tr.speed

        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]
        return [t for t in self.tracks if t.confirmed]


# ── module-level math helpers ──────────────────────────────────────────────────

def _read_xy(cloud):
    """Extract (x, y) tuples from a PointCloud2 with float32 x,y,z fields."""
    pts = []
    step = cloud.point_step
    for i in range(cloud.width):
        x, y, _z = struct.unpack_from("fff", cloud.data, i * step)
        pts.append((x, y))
    return pts


def _euclidean_cluster(points, tol, min_size):
    """Single-link Euclidean clustering (2D, region-growing). O(n^2)."""
    n = len(points)
    used = [False] * n
    clusters = []
    tol2 = tol * tol
    for i in range(n):
        if used[i]:
            continue
        used[i] = True
        stack = [i]
        comp = [i]
        while stack:
            j = stack.pop()
            jx, jy = points[j]
            for k in range(n):
                if used[k]:
                    continue
                dx = jx - points[k][0]
                dy = jy - points[k][1]
                if dx * dx + dy * dy <= tol2:
                    used[k] = True
                    stack.append(k)
                    comp.append(k)
        if len(comp) >= min_size:
            clusters.append([points[c] for c in comp])
    return clusters


def _wrap(a):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_from_quat(q):
    """Yaw (rad) from a geometry_msgs/Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def main(args=None):
    rclpy.init(args=args)
    node = GeoFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
