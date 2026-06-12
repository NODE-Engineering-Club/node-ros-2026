"""
Asket 2D Simulator — topics aligned with the Njord ROS2 stack
=============================================================
Publie :
  /lidar_driver/scan_raw   (sensor_msgs/LaserScan)   ← lidar_obstacle_node
  /odom                    (nav_msgs/Odometry)        ← EKF odom0
  /gps_driver/gps_raw      (sensor_msgs/NavSatFix)   ← navsat_transform_node
  /imu/data                (sensor_msgs/Imu)          ← EKF imu0

TF :
  map → odom        (identity — navsat_transform override ensuite)
  odom → base_link  (pose du bateau)
  base_link → lidar (static, valeurs URDF)
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

# ── Waypoints GPS (boucle fermée) ─────────────────────────────────────────
WAYPOINTS = [
    (41.3900, 2.1540),
    (41.3910, 2.1560),
    (41.3920, 2.1550),
    (41.3915, 2.1530),
    (41.3900, 2.1540),
]

# ── Obstacles mobiles ─────────────────────────────────────────────────────
OBSTACLES = [
    {"x":  5.0, "y":  0.0, "vx": -0.3, "vy":  0.1},
    {"x": -3.0, "y":  4.0, "vx":  0.2, "vy": -0.2},
    {"x":  2.0, "y": -5.0, "vx": -0.1, "vy":  0.3},
]

BOAT_SPEED   = 1.5   # m/s
WAYPOINT_TOL = 1.0   # m
DT           = 0.1   # s → 10 Hz
LIDAR_RANGE  = 10.0  # m
NUM_BEAMS    = 360


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2),
        w=math.cos(yaw / 2),
    )


def gps_to_meters(lat, lon, lat0, lon0):
    x = (lon - lon0) * math.cos(math.radians(lat0)) * 111320
    y = (lat - lat0) * 111320
    return x, y


class Simulator(Node):
    def __init__(self):
        super().__init__("asket_simulator")

        self._tf_br = TransformBroadcaster(self)

        # Topics alignés avec le stack Njord réel
        self.pub_scan = self.create_publisher(LaserScan, "/lidar_driver/scan_raw", 10)
        self.pub_odom = self.create_publisher(Odometry,  "/odom",                  10)
        self.pub_gps  = self.create_publisher(NavSatFix, "/gps_driver/gps_raw",    10)
        self.pub_imu  = self.create_publisher(Imu,       "/imu/data",              10)

        # État du bateau
        self._lat0, self._lon0 = WAYPOINTS[0]
        self._x    = 0.0
        self._y    = 0.0
        self._yaw  = 0.0
        self._wp_idx    = 1
        self._obstacles = [dict(o) for o in OBSTACLES]

        self.create_timer(DT, self._step)
        self.get_logger().info(
            "Asket 2D simulator started — topics aligned with Njord stack"
        )

    def _step(self):
        now = self.get_clock().now().to_msg()

        # Waypoint following
        wp_lat, wp_lon = WAYPOINTS[self._wp_idx]
        wp_x, wp_y = gps_to_meters(wp_lat, wp_lon, self._lat0, self._lon0)
        dx, dy = wp_x - self._x, wp_y - self._y
        dist = math.hypot(dx, dy)

        if dist < WAYPOINT_TOL:
            self._wp_idx = (self._wp_idx + 1) % len(WAYPOINTS)
            self.get_logger().info(f"Waypoint {self._wp_idx} atteint → suivant")
        else:
            self._yaw = math.atan2(dy, dx)
            self._x += BOAT_SPEED * math.cos(self._yaw) * DT
            self._y += BOAT_SPEED * math.sin(self._yaw) * DT

        # Obstacles
        for obs in self._obstacles:
            obs["x"] += obs["vx"] * DT
            obs["y"] += obs["vy"] * DT
            if abs(obs["x"]) > 15: obs["vx"] *= -1
            if abs(obs["y"]) > 15: obs["vy"] *= -1

        self._publish_lidar(now)
        self._publish_odom(now)
        self._publish_gps(now)
        self._publish_imu(now)
        self._publish_tf(now)

    # ── LiDAR ─────────────────────────────────────────────────────────────
    def _publish_lidar(self, now):
        ranges = [float("inf")] * NUM_BEAMS
        for i in range(NUM_BEAMS):
            angle = 2 * math.pi * i / NUM_BEAMS
            for obs in self._obstacles:
                ox = obs["x"] - self._x
                oy = obs["y"] - self._y
                obs_angle = math.atan2(oy, ox) - self._yaw
                a_diff = abs(math.atan2(
                    math.sin(angle - obs_angle),
                    math.cos(angle - obs_angle),
                ))
                if a_diff < math.radians(3):
                    d = math.hypot(ox, oy)
                    if d < LIDAR_RANGE:
                        ranges[i] = min(ranges[i], d)

        scan = LaserScan()
        scan.header.stamp    = now
        scan.header.frame_id = "lidar"
        scan.angle_min       = 0.0
        scan.angle_max       = 2 * math.pi
        scan.angle_increment = 2 * math.pi / NUM_BEAMS
        scan.range_min       = 0.2
        scan.range_max       = LIDAR_RANGE
        scan.scan_time       = DT
        scan.time_increment  = DT / NUM_BEAMS
        scan.ranges          = ranges
        self.pub_scan.publish(scan)

    # ── Odometry (/odom) ──────────────────────────────────────────────────
    def _publish_odom(self, now):
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.orientation = yaw_to_quaternion(self._yaw)
        odom.twist.twist.linear.x  = BOAT_SPEED
        # Covariances diagonales — données sim propres
        odom.pose.covariance[0]   = 0.1
        odom.pose.covariance[7]   = 0.1
        odom.pose.covariance[35]  = 0.1
        odom.twist.covariance[0]  = 0.1
        odom.twist.covariance[7]  = 0.1
        odom.twist.covariance[35] = 0.1
        self.pub_odom.publish(odom)

    # ── GPS (/gps_driver/gps_raw) ─────────────────────────────────────────
    def _publish_gps(self, now):
        lat = self._lat0 + self._y / 111320
        lon = self._lon0 + self._x / (111320 * math.cos(math.radians(self._lat0)))
        gps = NavSatFix()
        gps.header.stamp    = now
        gps.header.frame_id = "GPS"
        gps.latitude        = lat
        gps.longitude       = lon
        gps.altitude        = 0.0
        gps.status.status   = 0  # STATUS_FIX
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps.position_covariance[0] = 1.0
        gps.position_covariance[4] = 1.0
        gps.position_covariance[8] = 1.0
        self.pub_gps.publish(gps)

    # ── IMU (/imu/data) ───────────────────────────────────────────────────
    def _publish_imu(self, now):
        imu = Imu()
        imu.header.stamp    = now
        imu.header.frame_id = "px4"
        imu.orientation     = yaw_to_quaternion(self._yaw)
        imu.angular_velocity.z          = 0.0
        imu.linear_acceleration.x       = 0.0
        imu.linear_acceleration.y       = 0.0
        imu.linear_acceleration.z       = 9.81
        imu.orientation_covariance[8]         = 0.01
        imu.angular_velocity_covariance[8]    = 0.01
        imu.linear_acceleration_covariance[0] = 0.1
        imu.linear_acceleration_covariance[4] = 0.1
        imu.linear_acceleration_covariance[8] = 0.1
        self.pub_imu.publish(imu)

    # ── TF ────────────────────────────────────────────────────────────────
    def _publish_tf(self, now):
        # map → odom (identity bootstrap)
        t1 = TransformStamped()
        t1.header.stamp    = now
        t1.header.frame_id = "map"
        t1.child_frame_id  = "odom"
        t1.transform.rotation.w = 1.0
        self._tf_br.sendTransform(t1)

        # odom → base_link
        t2 = TransformStamped()
        t2.header.stamp    = now
        t2.header.frame_id = "odom"
        t2.child_frame_id  = "base_link"
        t2.transform.translation.x = self._x
        t2.transform.translation.y = self._y
        t2.transform.rotation      = yaw_to_quaternion(self._yaw)
        self._tf_br.sendTransform(t2)

        # base_link → lidar (valeurs exactes de l'URDF)
        t3 = TransformStamped()
        t3.header.stamp    = now
        t3.header.frame_id = "base_link"
        t3.child_frame_id  = "lidar"
        t3.transform.translation.x = -0.103585
        t3.transform.translation.z =  0.137275
        t3.transform.rotation.w    = 1.0
        self._tf_br.sendTransform(t3)


def main(args=None):
    rclpy.init(args=args)
    node = Simulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
