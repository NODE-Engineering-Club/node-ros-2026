import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Imu
from simple_pid import PID

STALE_TIMEOUT = 0.5  # seconds before a sensor is considered stale


class PidController(Node):
    def __init__(self):
        super().__init__("pid_controller")

        self.declare_parameter("kp_speed", 0.5)
        self.declare_parameter("ki_speed", 0.1)
        self.declare_parameter("kd_speed", 0.0)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.1)

        self._speed_pid = PID(
            self.get_parameter("kp_speed").value,
            self.get_parameter("ki_speed").value,
            self.get_parameter("kd_speed").value,
            setpoint=0,
            output_limits=(-1.0, 1.0),
            sample_time=0.05,
        )
        self._yaw_pid = PID(
            self.get_parameter("kp_yaw").value,
            self.get_parameter("ki_yaw").value,
            self.get_parameter("kd_yaw").value,
            setpoint=0,
            output_limits=(-1.0, 1.0),
            sample_time=0.05,
        )

        self._yaw_rate = 0.0
        self._speed = 0.0
        self._last_imu_time = None
        self._last_vel_time = None
        self._last_setpoint_time = None

        self.pub = self.create_publisher(Twist, "/control/effort", 10)
        self.create_subscription(Twist, "/control/setpoint", self._sp_cb, 10)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 10)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_body", self._vel_cb, 10)
        self.create_timer(0.05, self._control)  # 20 Hz
        self.add_on_set_parameters_callback(self._param_cb)

    def _param_cb(self, params):
        for p in params:
            if p.name == "kp_speed":   self._speed_pid.Kp = p.value
            elif p.name == "ki_speed": self._speed_pid.Ki = p.value
            elif p.name == "kd_speed": self._speed_pid.Kd = p.value
            elif p.name == "kp_yaw":   self._yaw_pid.Kp = p.value
            elif p.name == "ki_yaw":   self._yaw_pid.Ki = p.value
            elif p.name == "kd_yaw":   self._yaw_pid.Kd = p.value
        return SetParametersResult(successful=True)

    def _sp_cb(self, msg):
        self._speed_pid.setpoint = msg.linear.x
        self._yaw_pid.setpoint = msg.angular.z
        self._last_setpoint_time = self.get_clock().now()

    def _imu_cb(self, msg):
        self._yaw_rate = msg.angular_velocity.z
        self._last_imu_time = self.get_clock().now()

    def _vel_cb(self, msg):
        self._speed = msg.twist.linear.x
        self._last_vel_time = self.get_clock().now()

    def _control(self):
        now = self.get_clock().now()

        if self._last_imu_time is not None:
            if (now - self._last_imu_time).nanoseconds / 1e9 > STALE_TIMEOUT:
                self.get_logger().warn("IMU data stale", throttle_duration_sec=1.0)

        if self._last_vel_time is not None:
            if (now - self._last_vel_time).nanoseconds / 1e9 > STALE_TIMEOUT:
                self.get_logger().warn("Velocity data stale", throttle_duration_sec=1.0)

        # Nothing downstream of /cmd_vel (twist_mux, nav_to_pid) publishes an
        # explicit zero when its own input goes away — they're purely
        # event-driven and simply stop publishing. Without this check this
        # loop would keep outputting whatever effort the last real setpoint
        # produced forever, so a cancelled/aborted Nav2 goal or a boat_bt
        # command that just stops (rather than explicitly zeroing) would
        # leave the boat thrusting indefinitely. Treat "no setpoint yet" the
        # same as "setpoint gone stale": hold at zero.
        setpoint_stale = (
            self._last_setpoint_time is None
            or (now - self._last_setpoint_time).nanoseconds / 1e9 > STALE_TIMEOUT
        )
        if setpoint_stale:
            self._speed_pid.setpoint = 0.0
            self._yaw_pid.setpoint = 0.0
            # reset() alone doesn't touch .setpoint, and .setpoint alone
            # doesn't clear accumulated integral — need both so a stale
            # setpoint can't leave residual windup driving a nonzero output.
            self._speed_pid.reset()
            self._yaw_pid.reset()
            if self._last_setpoint_time is not None:
                self.get_logger().warn("Setpoint stale — holding at zero", throttle_duration_sec=1.0)

        effort = Twist()
        effort.linear.x = self._speed_pid(self._speed)
        effort.angular.z = self._yaw_pid(self._yaw_rate)
        self.pub.publish(effort)

        self.get_logger().debug(
            f"speed sp={self._speed_pid.setpoint:.2f} meas={self._speed:.2f} out={effort.linear.x:.3f} | "
            f"yaw sp={self._yaw_pid.setpoint:.2f} meas={self._yaw_rate:.2f} out={effort.angular.z:.3f}"
        )


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
