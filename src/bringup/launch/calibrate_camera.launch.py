from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_device", default_value="/dev/video0",
                              description="V4L2 device path for the camera"),
        DeclareLaunchArgument("size",   default_value="7x9",
                              description="Interior corner count of the checkerboard (cols x rows)"),
        DeclareLaunchArgument("square", default_value="0.02",
                              description="Size of one checkerboard square in metres"),

        Node(
            package="sensors",
            executable="camera_driver",
            name="camera_driver",
            parameters=[{
                "device":   LaunchConfiguration("camera_device"),
                "topic":    "/front_camera_driver/image_raw",
                "frame_id": "front_camera",
            }],
        ),
        Node(
            package="camera_calibration",
            executable="cameracalibrator",
            name="cameracalibrator",
            output="screen",
            ros_arguments=[
                "-r", "image:=/front_camera_driver/image_raw",
            ],
            arguments=[
                "--no-service-check",
                "--size", LaunchConfiguration("size"),
                "--square", LaunchConfiguration("square"),
            ],
        ),
    ])
