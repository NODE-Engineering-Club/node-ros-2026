import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

KP_SPEED, KI_SPEED, KD_SPEED = 0.5, 0.1, 0.0
KP_YAW, KI_YAW, KD_YAW = 1.0, 0.0, 0.1


class _PID:
    def __init__(self, kp, ki, kd, limits=(-1.0, 1.0)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limits = limits
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, error, now):
        if self._prev_time is None:
            self._prev_time = now
            return 0.0
        dt = now - self._prev_time
        if dt <= 0:
            return 0.0
        self._prev_time = now
        self._integral += error * dt
        deriv = (error - self._prev_error) / dt
        self._prev_error = error
        out = self.kp * error + self.ki * self._integral + self.kd * deriv
        return max(self.limits[0], min(self.limits[1], out))


class PidController(Node):
    def __init__(self):
        super().__init__("pid_controller")

        self._speed_pid = _PID(KP_SPEED, KI_SPEED, KD_SPEED)
        self._yaw_pid = _PID(KP_YAW, KI_YAW, KD_YAW)

        self._setpoint = Twist()
        self._yaw_rate = 0.0

        self.pub = self.create_publisher(Twist, "/control/effort", 10)
        self.create_subscription(Twist, "/control/setpoint", self._sp_cb, 10)
        self.create_subscription(Imu, "/imu_driver/imu_raw", self._imu_cb, 10)
        self.create_timer(0.05, self._control)  # 20 Hz

    def _sp_cb(self, msg):
        self._setpoint = msg

    def _imu_cb(self, msg):
        self._yaw_rate = msg.angular_velocity.z

    def _control(self):
        now = time.monotonic()
        effort = Twist()
        # Open-loop speed passthrough (no speed sensor yet — PID would wind up)
        effort.linear.x = self._setpoint.linear.x
        effort.angular.z = self._yaw_pid.compute(self._setpoint.angular.z - self._yaw_rate, now)
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
