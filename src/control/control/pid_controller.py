import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from simple_pid import PID

KP_SPEED, KI_SPEED, KD_SPEED = 0.5, 0.1, 0.0
KP_YAW, KI_YAW, KD_YAW = 1.0, 0.0, 0.1


class PidController(Node):
    def __init__(self):
        super().__init__("pid_controller")

        self._speed_pid = PID(KP_SPEED, KI_SPEED, KD_SPEED, setpoint=0, output_limits=(-1.0, 1.0))
        self._yaw_pid = PID(KP_YAW, KI_YAW, KD_YAW, setpoint=0, output_limits=(-1.0, 1.0))

        self._yaw_rate = 0.0
        self._speed = 0.0

        self.pub = self.create_publisher(Twist, "/control/effort", 10)
        self.create_subscription(Twist, "/control/setpoint", self._sp_cb, 10)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 10)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_body", self._vel_cb, 10)
        self.create_timer(0.05, self._control)  # 20 Hz

    def _sp_cb(self, msg):
        self._speed_pid.setpoint = msg.linear.x
        self._yaw_pid.setpoint = msg.angular.z

    def _imu_cb(self, msg):
        self._yaw_rate = msg.angular_velocity.z

    def _vel_cb(self, msg):
        self._speed = msg.twist.linear.x

    def _control(self):
        effort = Twist()
        effort.linear.x = self._speed_pid(self._speed)
        effort.angular.z = self._yaw_pid(self._yaw_rate)
        self.pub.publish(effort)


def main(args=None):
    rclpy.init(args=args)
    node = PidController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
