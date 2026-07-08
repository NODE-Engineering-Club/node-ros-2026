"""Publishes the calibrated lidar→camera_cal static TF from a YAML file.

Parameters:
  extrinsic_yaml  (string) — path to lidar_camera_extrinsic.yaml
  parent_frame    (string) — default "lidar"
  child_frame     (string) — default "front_camera_cal"
"""

import math

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


class ExtrinsicTFPublisher(Node):
    def __init__(self):
        super().__init__("extrinsic_tf_publisher")

        self.declare_parameter("extrinsic_yaml", "")
        self.declare_parameter("parent_frame",   "lidar")
        self.declare_parameter("child_frame",    "front_camera_cal")

        yaml_path    = self.get_parameter("extrinsic_yaml").get_parameter_value().string_value
        parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value
        child_frame  = self.get_parameter("child_frame").get_parameter_value().string_value

        if not yaml_path:
            self.get_logger().error("extrinsic_yaml parameter is empty — nothing to publish.")
            return

        with open(yaml_path) as f:
            data = yaml.safe_load(f)

        ext = data["lidar_to_camera"]
        x, y, z         = ext["x"], ext["y"], ext["z"]
        roll, pitch, yaw = ext["roll"], ext["pitch"], ext["yaw"]
        qx, qy, qz, qw  = _rpy_to_quaternion(roll, pitch, yaw)

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id  = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(t)

        err = data.get("reprojection_error_px", "?")
        self.get_logger().info(
            f"Published static TF: {parent_frame} → {child_frame}  "
            f"t=({x:.3f},{y:.3f},{z:.3f})  "
            f"rpy=({math.degrees(roll):.1f}°,{math.degrees(pitch):.1f}°,{math.degrees(yaw):.1f}°)  "
            f"[calibration reprojection error: {err} px]"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
