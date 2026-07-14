import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("description")
    urdf = xacro.process_file(desc_share + "/urdf/asket.urdf.xacro").toxml()

    return LaunchDescription([
        DeclareLaunchArgument("camera_device", default_value="/dev/video0",
                              description="V4L2 device path for the camera"),
        DeclareLaunchArgument("lidar_device",  default_value="/dev/ttyUSB0",
                              description="Serial device for the RPLidar"),

        # Publishes the static base_link->lidar / base_link->front_camera TF
        # from the URDF, used by collect_data to draw an approximate
        # (uncalibrated) LiDAR-scan-plane guide line on the camera image.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf}],
        ),
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
