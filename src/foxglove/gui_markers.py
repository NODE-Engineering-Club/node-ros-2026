#!/usr/bin/env python3
"""
gui_markers.py  --  ASKET 2.0 GUI helper (cardinal markers / buoys)

Republishes /obstacles/global (njord_msgs/ObstacleArray, already
geo-referenced + boat-relative by src/fusion/fusion/geo_fusion_node.py)
into the topics the mandatory Foxglove layout (ASKET_GUI_mandatory.json)
reads for jury-visible marker detection:

  /gui/cardinal_markers_3d   visualization_msgs/MarkerArray  (base_link)
      Colored per class_id, shown in the 3D LiDAR panel. This is the
      color-accurate view -- Foxglove's Map panel can only assign one
      fixed color per topic, so it can't recolor per-message.

  /gui/markers/marker_0..11  sensor_msgs/NavSatFix
      Fixed-slot redundancy for the 2D Map panel (mirrors the existing
      /competition/waypoints/wp_N pattern), single neutral color.

Class legend (NOT documented anywhere else in the repo -- pulled straight
from the model's embedded Ultralytics metadata:
  `strings models/yolo26n-seg-navier.onnx | grep names` ->
  {0: 'green', 1: 'red', 2: 'north', 3: 'east', 4: 'south', 5: 'west'}
0/1 are the port/starboard channel buoys, 2-5 are the four cardinal marks.
Obstacle.class_id is "unknown" for lidar-only (unconfirmed) detections.
"""

import math

import rclpy
from geometry_msgs.msg import Point
from njord_msgs.msg import ObstacleArray
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

NUM_MAP_SLOTS = 12  # color for these slots is set in the layout JSON's topicColors

# class_id (string) -> (r, g, b)
CLASS_COLORS = {
    "0": (0.0, 0.85, 0.0),   # green channel buoy
    "1": (0.9, 0.0, 0.0),    # red channel buoy
    "2": (0.9, 0.9, 0.0),    # north cardinal
    "3": (0.0, 0.85, 0.85),  # east cardinal
    "4": (0.2, 0.4, 1.0),    # south cardinal
    "5": (0.9, 0.0, 0.9),    # west cardinal
}
CLASS_LABELS = {
    "0": "green", "1": "red",
    "2": "north", "3": "east", "4": "south", "5": "west",
}
UNKNOWN_COLOR = (0.55, 0.55, 0.55)


class GuiMarkers(Node):
    def __init__(self):
        super().__init__("gui_markers")

        self.pub_markers_3d = self.create_publisher(
            MarkerArray, "/gui/cardinal_markers_3d", 10)
        self.map_slot_pubs = [
            self.create_publisher(NavSatFix, f"/gui/markers/marker_{i}", 10)
            for i in range(NUM_MAP_SLOTS)
        ]

        self.create_subscription(
            ObstacleArray, "/obstacles/global", self.on_obstacles, 10)

        self.get_logger().info(
            "gui_markers up — republishing /obstacles/global for the GUI")

    def on_obstacles(self, msg: ObstacleArray):
        self._publish_3d(msg)
        self._publish_map_slots(msg)

    def _publish_3d(self, msg: ObstacleArray):
        array = MarkerArray()
        clear = Marker()
        clear.header = msg.header
        clear.header.frame_id = "base_link"
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for i, obs in enumerate(msg.obstacles):
            rgb = CLASS_COLORS.get(obs.class_id, UNKNOWN_COLOR)
            label = CLASS_LABELS.get(obs.class_id, "unknown")
            bearing_rad = math.radians(obs.bearing_deg)
            x = obs.range_m * math.cos(bearing_rad)
            y = obs.range_m * math.sin(bearing_rad)  # FLU: +y = port

            body = Marker()
            body.header = msg.header
            body.header.frame_id = "base_link"
            body.ns = "cardinal_markers"
            body.id = i * 2
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position = Point(x=x, y=y, z=0.0)
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = max(obs.radius * 2.0, 0.5)
            body.scale.z = 1.0
            body.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.9)
            array.markers.append(body)

            text = Marker()
            text.header = body.header
            text.ns = "cardinal_markers_labels"
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=x, y=y, z=1.3)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.5
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f"{label} ({obs.range_m:.1f}m)"
            array.markers.append(text)

        self.pub_markers_3d.publish(array)

    def _publish_map_slots(self, msg: ObstacleArray):
        # Stable id->slot assignment; untracked (id == 0) obstacles fall
        # back to their array index so they still show up somewhere.
        # KNOWN LIMITATION: a slot's last position lingers on the Map panel
        # if that obstacle drops out of tracking -- NavSatFix has no
        # DELETEALL equivalent. The 3D panel is the always-current view.
        for i, obs in enumerate(msg.obstacles):
            slot = (obs.id if obs.id != 0 else i) % NUM_MAP_SLOTS
            fix = NavSatFix()
            fix.header = msg.header
            fix.latitude = obs.position.latitude
            fix.longitude = obs.position.longitude
            fix.altitude = obs.position.altitude
            self.map_slot_pubs[slot].publish(fix)


def main():
    rclpy.init()
    node = GuiMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
