import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensors.camera_driver import CameraDriver
from sensors.imu_gps_driver import ImuGpsDriver
from sensors.lidar_driver import LidarDriver


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    nodes = [CameraDriver(), LidarDriver(), ImuGpsDriver()]
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
