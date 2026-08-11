"""Synthetic scene publisher exercising geo_fusion_node's velocity_bearing_deg
(Task 9.2's marker-vessel identification relies on this field).

Publishes, at 10 Hz, a lidar-only cluster (no matching vision detection, so
the resulting obstacle's class_id stays "unknown" — same as the real marker
vessel, which has no YOLO class) moving at a known constant velocity
(VX, VY) in the base_link frame (x=forward, y=port/left), starting 5 m
ahead of the boat at t=0.

Expected fusion result once the tracker confirms (track_confirm_hits): a
single "unknown", lidar_confirmed obstacle whose velocity_bearing_deg
converges to atan2(VY, VX) in degrees.

Mirrors scene_publisher.py's structure (TF/GPS setup identical); the only
difference is a moving, unclassified cluster instead of a static, classified
one, and an empty (but still published, to keep the vision/lidar time
synchronizer firing) Detection2DArray each tick.
"""

import struct

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection2DArray

LAT0 = 63.4305
LON0 = 10.3951
ALT0 = 0.0

# Constant relative velocity of the synthetic moving obstacle, base_link
# frame (x=forward, y=port/left) — small enough per-tick displacement
# (~0.1 m/tick at 10 Hz) to stay within the tracker's default track_gate_m
# (1.5 m) for frame-to-frame association.
VX = 1.0
VY = 0.5

START_X = 5.0
START_Y = 0.0


class MovingScenePublisher(Node):
    def __init__(self):
        super().__init__("moving_scene_publisher")
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._tf = TransformBroadcaster(self)
        self._gps = self.create_publisher(NavSatFix, "/gps_driver/gps_raw", 10)
        self._det = self.create_publisher(Detection2DArray, "/yolo/detections", be_qos)
        self._lidar = self.create_publisher(PointCloud2, "/obstacles/lidar", 10)
        self._start_time = self.get_clock().now()
        self.create_timer(0.1, self._tick)

    def _tick(self):
        now = self.get_clock().now()
        elapsed_s = (now - self._start_time).nanoseconds * 1e-9
        stamp = now.to_msg()
        self._pub_tf(stamp)
        self._pub_gps(stamp)
        self._pub_det(stamp)
        self._pub_lidar(stamp, elapsed_s)

    def _pub_tf(self, stamp):
        chain = [("map", "odom"), ("odom", "base_link"), ("base_link", "lidar")]
        for parent, child in chain:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0  # identity
            self._tf.sendTransform(t)

    def _pub_gps(self, stamp):
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = "GPS"
        msg.latitude = LAT0
        msg.longitude = LON0
        msg.altitude = ALT0
        self._gps.publish(msg)

    def _pub_det(self, stamp):
        # Deliberately empty — no vision detection, so the lidar cluster
        # below gets class_id "unknown" (same as the real marker vessel,
        # which has no matching YOLO class). Still published (empty) so the
        # ApproximateTimeSynchronizer in geo_fusion_node keeps firing.
        arr = Detection2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = "front_camera"
        self._det.publish(arr)

    def _pub_lidar(self, stamp, elapsed_s):
        cx = START_X + VX * elapsed_s
        cy = START_Y + VY * elapsed_s
        cluster = [
            (cx, cy),
            (cx + 0.05, cy + 0.05),
            (cx - 0.05, cy - 0.05),
            (cx + 0.04, cy - 0.04),
        ]

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = "lidar"
        msg.height = 1
        msg.width = len(cluster)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(cluster)
        msg.data = b"".join(struct.pack("fff", x, y, 0.0) for x, y in cluster)
        msg.is_dense = True
        self._lidar.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MovingScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
