"""Synthetic scene publisher for exercising geo_fusion_node without Gazebo/YOLO.

Publishes, at 10 Hz, a deterministic world:
  * TF:   map -> odom -> base_link -> lidar  (all identity; boat at the origin,
          facing +x = East)
  * /gps_driver/gps_raw    NavSatFix at a fixed datum (Trondheim)
  * /yolo/detections       one detection, class "2", centred (bearing 0 = ahead)
  * /obstacles/lidar       a small lidar cluster 5 m straight ahead

Expected fusion result: a single obstacle, class "2", lidar_confirmed, at
~5 m East of the boat, i.e. roughly (lat0, lon0 + 5 m east).

Reused by both the integration test and the rosbag recording demo.
"""

import struct

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, PointCloud2, PointField
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

LAT0 = 63.4305
LON0 = 10.3951
ALT0 = 0.0

# lidar cluster: 5 m straight ahead (+x), a few points within cluster tolerance
CLUSTER = [(5.0, 0.0), (5.05, 0.05), (4.95, -0.05), (5.0, 0.04)]


class ScenePublisher(Node):
    def __init__(self):
        super().__init__("scene_publisher")
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._tf = TransformBroadcaster(self)
        self._gps = self.create_publisher(NavSatFix, "/gps_driver/gps_raw", 10)
        self._det = self.create_publisher(Detection2DArray, "/yolo/detections", be_qos)
        self._lidar = self.create_publisher(PointCloud2, "/obstacles/lidar", 10)
        self.create_timer(0.1, self._tick)

    def _tick(self):
        now = self.get_clock().now().to_msg()
        self._pub_tf(now)
        self._pub_gps(now)
        self._pub_det(now)
        self._pub_lidar(now)

    def _pub_tf(self, now):
        chain = [("map", "odom"), ("odom", "base_link"), ("base_link", "lidar")]
        for parent, child in chain:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0  # identity
            self._tf.sendTransform(t)

    def _pub_gps(self, now):
        msg = NavSatFix()
        msg.header.stamp = now
        msg.header.frame_id = "GPS"
        msg.latitude = LAT0
        msg.longitude = LON0
        msg.altitude = ALT0
        self._gps.publish(msg)

    def _pub_det(self, now):
        arr = Detection2DArray()
        arr.header.stamp = now
        arr.header.frame_id = "front_camera"
        det = Detection2D()
        det.header = arr.header
        det.bbox.center.position.x = 320.0  # image centre -> bearing 0 (ahead)
        det.bbox.center.position.y = 240.0
        det.bbox.size_x = 40.0
        det.bbox.size_y = 40.0
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "2"
        hyp.hypothesis.score = 0.9
        det.results.append(hyp)
        arr.detections.append(det)
        self._det.publish(arr)

    def _pub_lidar(self, now):
        msg = PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = "lidar"
        msg.height = 1
        msg.width = len(CLUSTER)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(CLUSTER)
        msg.data = b"".join(struct.pack("fff", x, y, 0.0) for x, y in CLUSTER)
        msg.is_dense = True
        self._lidar.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
