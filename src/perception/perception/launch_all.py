import rclpy
from rclpy.executors import MultiThreadedExecutor
from perception.lidar_obstacle_node import LidarObstacleNode
from perception.fusion_node import FusionNode


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    nodes = [LidarObstacleNode(), FusionNode()]
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
