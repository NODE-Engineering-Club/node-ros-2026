# fusion

Geo-referenced sensor fusion for the Njord 2026 USV.

`geo_fusion_node` time-matches **vision**, **lidar**, and **GPS**, then publishes a
list of obstacles anchored to the global GPS frame — each with a class/colour
label and absolute coordinates — plus the boat's own absolute pose. This is the
single message the decision / path-planning node consumes.

It runs **alongside** `perception/fusion_node` (which outputs a label-less
`PointCloud2` in `base_link` for the Nav2 costmap) so the two can be compared.

## Topics

| Dir | Topic | Type |
|-----|-------|------|
| sub | `/yolo/detections` | `vision_msgs/Detection2DArray` |
| sub | `/obstacles/lidar` | `sensor_msgs/PointCloud2` |
| sub | `/gps_driver/gps_raw` | `sensor_msgs/NavSatFix` |
| pub | `/obstacles/global` | `njord_msgs/ObstacleArray` |

`ObstacleArray` = `header` (frame `map`) + `boat_position` (GeoPoint) +
`boat_heading` + `Obstacle[]`. Each `Obstacle` = `id` (tracked), `class_id`,
`confidence`, `position` (absolute lat/lon), `position_map` (map-frame x/y for
planners), `radius`, `lidar_confirmed`.

## Pipeline

1. `message_filters` approximately time-matches detections + lidar.
2. Euclidean clustering turns the lidar cloud into discrete objects.
3. Full-TF transform (rotation included) of each cluster into `base_link` (for
   bearing) and `map` (for position).
4. Bearing association (camera HFOV) attaches a class/colour to each cluster;
   unmatched clusters become `unknown` obstacles, unmatched detections become
   bearing-only estimates.
5. Nearest-neighbour tracker assigns a persistent `id` and smooths position.
6. Map points → absolute lat/lon via a local-tangent (ENU) anchor on the GPS fix.

Key parameters (see `geo_fusion_node.py` for the full list and defaults):
`cluster_tolerance`, `bearing_match_tol_deg`, `sync_slop`, `confidence_min`,
`emit_vision_only`, `vision_only_distance`, `track_gating_distance`,
`track_smoothing`, `track_min_hits`, `track_max_misses`.

## Assumptions / future work

- The `map` frame is assumed ENU-aligned (matches the sim world + navsat config).
  A more rigorous alternative is the `/toLL` service from `navsat_transform_node`
  (needs async handling to avoid executor deadlock).
- Association is bearing-based, not full 3D projection / seg-mask sampling.
- Tracker is greedy NN + EMA (no motion model); a Kalman/JPDA upgrade is the
  next step for fast or crossing targets.

## Testing

Deterministic in-process integration test (no Gazebo / YOLO needed):

```bash
colcon build --packages-select njord_msgs fusion --symlink-install
source install/setup.bash
python3 src/fusion/test/test_geo_fusion.py     # exit 0 = pass
```

Record + replay a synthetic rosbag of the input topics:

```bash
# record
python3 src/fusion/test/scene_publisher.py &
ros2 bag record -o /tmp/fusion_demo /tf /gps_driver/gps_raw /yolo/detections /obstacles/lidar
# replay through the node
ros2 run fusion geo_fusion_node &
ros2 bag play /tmp/fusion_demo --loop &
ros2 topic echo /obstacles/global
```

## Launch

Brought up by `bringup/njord.launch.py` under `enable_geo_fusion` (default true):

```bash
ros2 launch bringup njord.launch.py use_sim:=true enable_sensors:=false enable_mavros:=false
```
