import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

MAX_SPEED = 2.0     # m/s
MAX_YAW_RATE = 1.0  # rad/s


class NavToPid(Node):
    def __init__(self):
        super().__init__("nav_to_pid")
        self.pub = self.create_publisher(Twist, "/control/setpoint", 10)
        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)

    def _cb(self, msg):
        sp = Twist()
        sp.linear.x = max(-MAX_SPEED, min(MAX_SPEED, msg.linear.x))
        sp.angular.z = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, msg.angular.z))
        self.pub.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = NavToPid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
