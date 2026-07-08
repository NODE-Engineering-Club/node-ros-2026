from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_device", default_value="/dev/video0",
                              description="V4L2 device path for the camera"),
        DeclareLaunchArgument("lidar_device",  default_value="/dev/ttyUSB0",
                              description="Serial device for the RPLidar"),

        Node(
            package="sensors",
            executable="camera_driver",
            name="camera_driver",
            parameters=[{
                "device":          LaunchConfiguration("camera_device"),
                "topic":           "/front_camera_driver/image_raw",
                "frame_id":        "front_camera",
                "camera_info_url": "package://bringup/config/front_camera.yaml",
            }],
        ),
        Node(
            package="sensors",
            executable="lidar_driver",
            name="lidar_driver",
            parameters=[{"device": LaunchConfiguration("lidar_device")}],
        ),
        Node(
            package="calibration",
            executable="scan_to_cloud",
            name="scan_to_cloud",
        ),
        Node(
            package="calibration",
            executable="collect_data",
            name="collect_data",
            output="screen",
        ),
    ])
