"""Publishes the calibrated lidar→camera_cal static TF from a YAML file.

Parameters:
  extrinsic_yaml  (string) — path to lidar_camera_extrinsic.yaml
  parent_frame    (string) — default "lidar"
  child_frame     (string) — default "front_camera_cal"
"""

import math

import numpy as np
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


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll) — matches calibrate.py's
    _rvec_to_rpy / this module's _rpy_to_quaternion convention."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _matrix_to_rpy(R: np.ndarray) -> tuple:
    """Inverse of _rpy_to_matrix, matching calibrate.py's _rvec_to_rpy."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll  = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = math.atan2(R[1, 0], R[0, 0])
    else:
        roll  = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = 0.0
    return roll, pitch, yaw


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
        x, y, z          = ext["x"], ext["y"], ext["z"]
        roll, pitch, yaw = ext["roll"], ext["pitch"], ext["yaw"]

        # calibrate.py's solvePnP gives (R, t) meaning p_camera = R @ p_lidar + t
        # (a lidar-frame point expressed in camera-optical coordinates). A TF
        # broadcast with parent=lidar, child=front_camera_cal means the
        # opposite: p_lidar = T @ p_camera (child-to-parent). Broadcasting
        # (R, t) unmodified would silently invert the transform's meaning —
        # every downstream point would be projected backwards. Broadcast the
        # mathematical inverse instead: R_inv = R^T, t_inv = -R^T @ t.
        R = _rpy_to_matrix(roll, pitch, yaw)
        t_vec = np.array([x, y, z])
        R_inv = R.T
        t_inv = -R_inv @ t_vec
        roll_i, pitch_i, yaw_i = _matrix_to_rpy(R_inv)
        qx, qy, qz, qw = _rpy_to_quaternion(roll_i, pitch_i, yaw_i)

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id  = child_frame
        t.transform.translation.x = float(t_inv[0])
        t.transform.translation.y = float(t_inv[1])
        t.transform.translation.z = float(t_inv[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(t)

        err = data.get("reprojection_error_px", "?")
        self.get_logger().info(
            f"Published static TF: {parent_frame} → {child_frame}  "
            f"t=({t_inv[0]:.3f},{t_inv[1]:.3f},{t_inv[2]:.3f})  "
            f"rpy=({math.degrees(roll_i):.1f}°,{math.degrees(pitch_i):.1f}°,{math.degrees(yaw_i):.1f}°)  "
            f"(inverted from solved R,t=({x:.3f},{y:.3f},{z:.3f}), "
            f"rpy=({math.degrees(roll):.1f}°,{math.degrees(pitch):.1f}°,{math.degrees(yaw):.1f}°))  "
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
