"""
Asket 2D simulator driven by the final Nav2 velocity command.

Publishes:
  /lidar_driver/scan_raw   sensor_msgs/LaserScan
  /odom                    nav_msgs/Odometry
  /gps_driver/gps_raw      sensor_msgs/NavSatFix
  /imu/data                sensor_msgs/Imu

Subscribes:
  /cmd_vel                 geometry_msgs/Twist

TF:
  map -> odom
  base_link -> lidar

The EKF is responsible for publishing:
  odom -> base_link
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster


GPS_ORIGIN_LATITUDE = 41.3900
GPS_ORIGIN_LONGITUDE = 2.1540

OBSTACLES = [
    {
        "x": 5.0,
        "y": 0.0,
        "vx": -0.3,
        "vy": 0.1,
    },
    {
        "x": -3.0,
        "y": 4.0,
        "vx": 0.2,
        "vy": -0.2,
    },
    {
        "x": 2.0,
        "y": -5.0,
        "vx": -0.1,
        "vy": 0.3,
    },
]

DT = 0.1
COMMAND_TIMEOUT = 0.5

LIDAR_RANGE = 10.0
NUM_BEAMS = 360


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a planar yaw angle into a quaternion."""

    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


class Simulator(Node):
    """Simple planar boat simulator controlled through /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("asket_simulator")

        self._tf_broadcaster = TransformBroadcaster(self)

        self._scan_publisher = self.create_publisher(
            LaserScan,
            "/lidar_driver/scan_raw",
            10,
        )

        self._odom_publisher = self.create_publisher(
            Odometry,
            "/odom",
            10,
        )

        self._gps_publisher = self.create_publisher(
            NavSatFix,
            "/gps_driver/gps_raw",
            10,
        )

        self._imu_publisher = self.create_publisher(
            Imu,
            "/imu/data",
            10,
        )

        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self._cmd_vel_callback,
            10,
        )

        self._latitude_origin = GPS_ORIGIN_LATITUDE
        self._longitude_origin = GPS_ORIGIN_LONGITUDE

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._linear_velocity = 0.0
        self._angular_velocity = 0.0

        self._last_command_time = self.get_clock().now()

        self._obstacles = [
            dict(obstacle)
            for obstacle in OBSTACLES
        ]

        self.create_timer(
            DT,
            self._step,
        )

        self.get_logger().info(
            "Asket 2D simulator started — movement controlled by /cmd_vel"
        )

    def _cmd_vel_callback(self, message: Twist) -> None:
        """Store the latest commanded linear and angular velocities."""

        self._linear_velocity = float(
            message.linear.x
        )

        self._angular_velocity = float(
            message.angular.z
        )

        self._last_command_time = self.get_clock().now()

    def _step(self) -> None:
        """Advance the simulation and publish all sensor data."""

        now = self.get_clock().now()
        now_message = now.to_msg()

        elapsed_since_command = (
            now - self._last_command_time
        ).nanoseconds / 1e9

        if elapsed_since_command > COMMAND_TIMEOUT:
            self._linear_velocity = 0.0
            self._angular_velocity = 0.0

        self._integrate_boat_motion()
        self._update_obstacles()

        self._publish_lidar(now_message)
        self._publish_odom(now_message)
        self._publish_gps(now_message)
        self._publish_imu(now_message)
        self._publish_tf(now_message)

    def _integrate_boat_motion(self) -> None:
        """Update the planar boat pose from the velocity command."""

        self._yaw += self._angular_velocity * DT

        self._yaw = math.atan2(
            math.sin(self._yaw),
            math.cos(self._yaw),
        )

        self._x += (
            self._linear_velocity
            * math.cos(self._yaw)
            * DT
        )

        self._y += (
            self._linear_velocity
            * math.sin(self._yaw)
            * DT
        )

    def _update_obstacles(self) -> None:
        """Move the simple simulated obstacles."""

        for obstacle in self._obstacles:
            obstacle["x"] += obstacle["vx"] * DT
            obstacle["y"] += obstacle["vy"] * DT

            if abs(obstacle["x"]) > 15.0:
                obstacle["vx"] *= -1.0

            if abs(obstacle["y"]) > 15.0:
                obstacle["vy"] *= -1.0

    def _publish_lidar(self, now) -> None:
        """Publish a basic 360-degree simulated laser scan."""

        ranges = [
            float("inf")
        ] * NUM_BEAMS

        for beam_index in range(NUM_BEAMS):
            beam_angle = (
                2.0
                * math.pi
                * beam_index
                / NUM_BEAMS
            )

            for obstacle in self._obstacles:
                obstacle_x = obstacle["x"] - self._x
                obstacle_y = obstacle["y"] - self._y

                obstacle_angle = (
                    math.atan2(
                        obstacle_y,
                        obstacle_x,
                    )
                    - self._yaw
                )

                angle_difference = abs(
                    math.atan2(
                        math.sin(
                            beam_angle
                            - obstacle_angle
                        ),
                        math.cos(
                            beam_angle
                            - obstacle_angle
                        ),
                    )
                )

                if angle_difference >= math.radians(3.0):
                    continue

                distance = math.hypot(
                    obstacle_x,
                    obstacle_y,
                )

                if distance < LIDAR_RANGE:
                    ranges[beam_index] = min(
                        ranges[beam_index],
                        distance,
                    )

        scan = LaserScan()

        scan.header.stamp = now
        scan.header.frame_id = "lidar"

        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (
            2.0
            * math.pi
            / NUM_BEAMS
        )

        scan.range_min = 0.2
        scan.range_max = LIDAR_RANGE
        scan.scan_time = DT
        scan.time_increment = DT / NUM_BEAMS
        scan.ranges = ranges

        self._scan_publisher.publish(scan)

    def _publish_odom(self, now) -> None:
        """Publish simulated planar odometry."""

        odometry = Odometry()

        odometry.header.stamp = now
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"

        odometry.pose.pose.position.x = self._x
        odometry.pose.pose.position.y = self._y
        odometry.pose.pose.orientation = yaw_to_quaternion(
            self._yaw
        )

        odometry.twist.twist.linear.x = (
            self._linear_velocity
        )
        odometry.twist.twist.angular.z = (
            self._angular_velocity
        )

        odometry.pose.covariance[0] = 0.1
        odometry.pose.covariance[7] = 0.1
        odometry.pose.covariance[35] = 0.1

        odometry.twist.covariance[0] = 0.1
        odometry.twist.covariance[7] = 0.1
        odometry.twist.covariance[35] = 0.1

        self._odom_publisher.publish(odometry)

    def _publish_gps(self, now) -> None:
        """Publish GPS calculated from the current simulated pose."""

        latitude = (
            self._latitude_origin
            + self._y / 111320.0
        )

        longitude = (
            self._longitude_origin
            + self._x
            / (
                111320.0
                * math.cos(
                    math.radians(
                        self._latitude_origin
                    )
                )
            )
        )

        gps = NavSatFix()

        gps.header.stamp = now
        gps.header.frame_id = "GPS"

        gps.latitude = latitude
        gps.longitude = longitude
        gps.altitude = 0.0

        gps.status.status = 0

        gps.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )

        gps.position_covariance[0] = 1.0
        gps.position_covariance[4] = 1.0
        gps.position_covariance[8] = 1.0

        self._gps_publisher.publish(gps)

    def _publish_imu(self, now) -> None:
        """Publish simulated orientation and angular velocity."""

        imu = Imu()

        imu.header.stamp = now
        imu.header.frame_id = "px4"

        imu.orientation = yaw_to_quaternion(
            self._yaw
        )

        imu.angular_velocity.z = (
            self._angular_velocity
        )

        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81

        imu.orientation_covariance[8] = 0.01
        imu.angular_velocity_covariance[8] = 0.01

        imu.linear_acceleration_covariance[0] = 0.1
        imu.linear_acceleration_covariance[4] = 0.1
        imu.linear_acceleration_covariance[8] = 0.1

        self._imu_publisher.publish(imu)

    def _publish_tf(self, now) -> None:
        """
        Publish transforms owned by the simulator.

        The EKF publishes odom -> base_link, so the simulator must not
        publish the same transform.
        """

        map_to_odom = TransformStamped()

        map_to_odom.header.stamp = now
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0

        self._tf_broadcaster.sendTransform(
            map_to_odom
        )

        base_to_lidar = TransformStamped()

        base_to_lidar.header.stamp = now
        base_to_lidar.header.frame_id = "base_link"
        base_to_lidar.child_frame_id = "lidar"

        base_to_lidar.transform.translation.x = -0.103585
        base_to_lidar.transform.translation.z = 0.137275
        base_to_lidar.transform.rotation.w = 1.0

        self._tf_broadcaster.sendTransform(
            base_to_lidar
        )


def main(args=None) -> None:
    """Run the simulator node."""

    rclpy.init(args=args)

    node = Simulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()