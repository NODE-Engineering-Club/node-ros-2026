"""
Pico Serial Bridge
Écoute /control/effort (Twist) et envoie les commandes moteur
au Raspberry Pi Pico via USB série.
"""
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

SERIAL_PORT    = "/dev/ttyACM0"
BAUD_RATE      = 115200
FAILSAFE_S     = 0.5


class PicoBridge(Node):
    def __init__(self):
        super().__init__("pico_bridge")

        self.declare_parameter("port", SERIAL_PORT)
        self.declare_parameter("baud", BAUD_RATE)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Pico connecté sur {port} à {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Impossible d'ouvrir {port}: {e}")
            self._ser = None

        self.create_subscription(Twist, "/control/effort", self._cb, 10)
        self.create_timer(FAILSAFE_S, self._failsafe)
        self._last_cmd = self.get_clock().now()
        self.get_logger().info("En attente de commandes sur /control/effort ...")

    def _cb(self, msg):
        left  = msg.linear.x - msg.angular.z
        right = msg.linear.x + msg.angular.z
        left  = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        self.get_logger().info(f"Commande reçue → L:{left:.3f} R:{right:.3f}")
        self._send(left, right)
        self._last_cmd = self.get_clock().now()

    def _failsafe(self):
        elapsed = (self.get_clock().now() - self._last_cmd).nanoseconds / 1e9
        if elapsed > FAILSAFE_S:
            self._send(0.0, 0.0)

    def _send(self, left, right):
        if self._ser is None or not self._ser.is_open:
            return
        cmd = f"{left:.3f},{right:.3f}\n"
        try:
            self._ser.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().warn(f"Erreur série: {e}")

    def destroy_node(self):
        self._send(0.0, 0.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PicoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
