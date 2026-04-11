"""Converts control effort to MAVROS RC override for ArduPilot."""

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node

RC_CENTER = 1500
RC_RANGE = 400  # +/-400 around center → 1100-1900
CHAN_STEERING = 0  # RC channel 1 (0-indexed)
CHAN_THROTTLE = 2  # RC channel 3 (0-indexed)
CHAN_NOCHANGE = 65535


class ActuatorDriver(Node):
    def __init__(self):
        super().__init__("actuator_driver")
        self.pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.create_subscription(Twist, "/control/effort", self._cb, 10)

    def _cb(self, msg):
        rc = OverrideRCIn()
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
