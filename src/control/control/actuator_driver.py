"""Converts control effort to Gazebo cmd_vel (sim) or MAVROS RC override (hardware)."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

RC_CENTER = 1500
RC_RANGE = 400  # +/-400 around center → 1100-1900
CHAN_STEERING = 0  # RC channel 1 (0-indexed)
CHAN_THROTTLE = 2  # RC channel 3 (0-indexed)
CHAN_NOCHANGE = 65535

MAX_SPEED = 2.0      # m/s — must match nav_to_pid.MAX_SPEED
MAX_YAW_RATE = 1.0   # rad/s — must match nav_to_pid.MAX_YAW_RATE
EFFORT_DEADBAND = 0.01  # ignore effort noise below 1% — prevents IMU noise from drifting boat


class ActuatorDriver(Node):
    def __init__(self):
        super().__init__("actuator_driver")
        self.declare_parameter("use_sim", False)
        self._use_sim = self.get_parameter("use_sim").value

        if self._use_sim:
            # Sim: scale normalized effort to velocity and feed Gazebo VelocityControl
            self.pub = self.create_publisher(Twist, "/cmd_vel_gz", 10)
        else:
            # Hardware: map effort to MAVROS RC channel override
            from mavros_msgs.msg import OverrideRCIn  # noqa: PLC0415
            self._OverrideRCIn = OverrideRCIn
            self.pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)

        self.create_subscription(Twist, "/control/effort", self._cb, 10)

    def _cb(self, msg):
        if self._use_sim:
            vel = Twist()
            vel.linear.x = msg.linear.x * MAX_SPEED if abs(msg.linear.x) > EFFORT_DEADBAND else 0.0
            vel.angular.z = msg.angular.z * MAX_YAW_RATE if abs(msg.angular.z) > EFFORT_DEADBAND else 0.0
            self.pub.publish(vel)
        else:
            rc = self._OverrideRCIn()
            rc.channels = [CHAN_NOCHANGE] * 18
            rc.channels[CHAN_STEERING] = int(RC_CENTER + msg.angular.z * RC_RANGE)
            rc.channels[CHAN_THROTTLE] = int(RC_CENTER + msg.linear.x * RC_RANGE)
            self.pub.publish(rc)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
