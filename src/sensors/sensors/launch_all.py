import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensors.camera_driver import CameraDriver
from sensors.imu_gps_driver import ImuGpsDriver

# LiDAR is no longer run in-process here: the RPLIDAR S2 is driven by
# sllidar_ros2's own C++ executable (sllidar_node), launched separately.


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    nodes = [CameraDriver(), ImuGpsDriver()]
    for n in nodes:
        executor.add_node(n)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()
