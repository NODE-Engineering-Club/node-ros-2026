import rclpy
from rclpy.executors import MultiThreadedExecutor
from control.nav_to_pid import NavToPid
from control.pid_controller import PidController
from control.actuator_driver import ActuatorDriver


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    nodes = [NavToPid(), PidController(), ActuatorDriver()], BuoyDecisionNode()
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
