"""ROS 2 wrapper around the fake Omniscan 3D.

Runs the simulated sonar as a network device so ``omniscan_bridge`` connects to
it exactly as it would to the real unit. Kept separate from ``sim_node`` because
in a real deployment this process must not exist, and a launch file should be
able to say so by simply not starting it.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from asket_sim.core.fake_sonar_server import FakeSonarServer
from asket_sim.core.world import SimWorld, WorldConfig


class FakeSonarNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_sonar")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 62312)
        self.declare_parameter("ping_rate_hz", 5.0)
        self.declare_parameter("points_per_ping", 256)
        self.declare_parameter("range_m", 30.0)
        self.declare_parameter("time_scale", 1.0)

        cfg = WorldConfig()
        cfg.sonar.ping_rate_hz = float(self.get_parameter("ping_rate_hz").value)
        cfg.sonar.points_per_ping = int(self.get_parameter("points_per_ping").value)
        cfg.sonar.range_setting_m = float(self.get_parameter("range_m").value)

        self.server = FakeSonarServer(
            SimWorld(cfg),
            host=self.get_parameter("host").value,
            port=int(self.get_parameter("port").value),
            time_scale=float(self.get_parameter("time_scale").value),
        )
        self.server.start()
        self.get_logger().info(
            f"simulated Omniscan 3D listening on "
            f"{self.server.host}:{self.server.bound_port}"
        )
        self.create_timer(5.0, self._report)

    def _report(self) -> None:
        self.get_logger().debug(
            f"{self.server.frames_sent} frames sent to {len(self.server.clients)} client(s); "
            f"ntp url = {self.server.ntp_url!r}"
        )

    def destroy_node(self) -> bool:
        self.server.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeSonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
