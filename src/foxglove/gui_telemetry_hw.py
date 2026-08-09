#!/usr/bin/env python3
"""
gui_telemetry_hw.py  --  ASKET 2.0 GUI helper (REAL data, MAVROS-fed)

Republishes real Pixhawk/MAVROS data into the topic names the mandatory
Foxglove layout (ASKET_GUI_mandatory.json) reads. Nothing is faked.

Publishes:
  /heading             std_msgs/Float64   real compass heading (deg, 0-360)
  /cog                 std_msgs/Float64   course over ground   (deg, 0=N, 90=E)
  /sog                 std_msgs/Float64   speed over ground    (knots)
  /battery_percentage  std_msgs/Float64   real battery         (%)
  /vehicle/status      std_msgs/String    AUTONOMOUS / REMOTE / STANDBY / OUT_OF_CONTROL

Subscribes (all MAVROS):
  /mavros/global_position/compass_hdg   std_msgs/Float64           heading
  /mavros/battery                       sensor_msgs/BatteryState   battery
  /mavros/state                         mavros_msgs/State          flight mode
  /mavros/global_position/raw/gps_vel   geometry_msgs/TwistStamped ENU velocity
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State

SPEED_DEADBAND = 0.05  # m/s; below this hold last COG

# ArduPilot Rover mode -> jury-facing status
MODE_MAP = {
    "MANUAL": "REMOTE",
    "ACRO": "REMOTE",
    "HOLD": "STANDBY",
    "AUTO": "AUTONOMOUS",
    "GUIDED": "AUTONOMOUS",
    "RTL": "AUTONOMOUS",
    "SMART_RTL": "AUTONOMOUS",
    "LOITER": "STANDBY",
}


class GuiTelemetryHW(Node):
    def __init__(self):
        super().__init__("gui_telemetry_hw")

        self.pub_heading = self.create_publisher(Float64, "/heading", 10)
        self.pub_cog = self.create_publisher(Float64, "/cog", 10)
        self.pub_sog = self.create_publisher(Float64, "/sog", 10)
        self.pub_batt = self.create_publisher(Float64, "/battery_percentage", 10)
        self.pub_status = self.create_publisher(String, "/vehicle/status", 10)

        # MAVROS sensor topics are best-effort -> use sensor QoS
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg",
                                 self.on_hdg, qos_profile_sensor_data)
        self.create_subscription(BatteryState, "/mavros/battery",
                                 self.on_batt, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/mavros/global_position/raw/gps_vel",
                                 self.on_vel, qos_profile_sensor_data)
        # /mavros/state is reliable/latched -> default QoS
        self.create_subscription(State, "/mavros/state", self.on_state, 10)

        self.last_cog = 0.0
        self.connected = False
        self.get_logger().info("gui_telemetry_hw up — republishing real MAVROS data")

    def on_hdg(self, msg: Float64):
        self.pub_heading.publish(Float64(data=msg.data))  # already 0-360 deg

    def on_batt(self, msg: BatteryState):
        # BatteryState.percentage is 0.0-1.0; -1 means "unknown"
        pct = msg.percentage * 100.0 if msg.percentage >= 0 else 0.0
        self.pub_batt.publish(Float64(data=pct))

    def on_vel(self, msg: TwistStamped):
        east = msg.twist.linear.x   # ENU: x=East, y=North
        north = msg.twist.linear.y
        speed = math.hypot(east, north)
        self.pub_sog.publish(Float64(data=speed * 1.94384))  # m/s -> knots
        if speed > SPEED_DEADBAND:
            self.last_cog = math.degrees(math.atan2(east, north)) % 360.0
        self.pub_cog.publish(Float64(data=self.last_cog))

    def on_state(self, msg: State):
        self.connected = msg.connected
        if not msg.connected:
            status = "OUT_OF_CONTROL"
        elif not msg.armed:
            status = "STANDBY"
        else:
            status = MODE_MAP.get(msg.mode, "STANDBY")
        self.pub_status.publish(String(data=status))


def main():
    rclpy.init()
    node = GuiTelemetryHW()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
