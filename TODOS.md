# Njord 2026 — Outstanding Gaps

## HIGH PRIORITY — Blocking Water Test (Wednesday)

- [ ] **Disable RPP rotate-to-heading**
  `bringup/config/nav2_params.yaml:32` enables in-place rotation
  (`rotate_to_heading_angular_vel: 0.5`). Blocked on confirming `FRAME_TYPE`
  on the Pixhawk via QGC. If `FRAME_TYPE=2` (skid-steer), leave enabled and
  also enable `allow_reversing: true` and add `spin`/`back_up` to
  `behavior_server`. If `FRAME_TYPE=0` (normal steering), set
  `use_rotate_to_heading: false` and `allow_reversing: false`.

- [ ] **Stand-test dry-run before water**
  1. Boat on a stand, FCU + RPi + thrusters connected.
  2. Launch the stack. Confirm: `/mavros/state.connected=true`,
     `/imu_driver/imu_raw` + `/gps_driver/gps_raw` publishing,
     `/odometry/filtered` position updates when boat is physically carried
     a few meters outside.
  3. Call `/mission/start` with a waypoint ~10 m away. Confirm thrusters
     spin in a direction that would drive toward the goal.
  4. Call `/mission/abort`. Confirm thrusters stop within 2 s.

## Navigation (Docking)

- [ ] **Restore `opennav_docking` for the docking challenge**
  The Nav2 docking server (`opennav_docking`) was intentionally omitted from
  `bringup/launch/njord.launch.py` and `Containerfile` because the package
  isn't installed and including it would crash the lifecycle manager. We need
  it back for the docking challenge. Required:
  1. Add `ros-jazzy-opennav-docking` to `Containerfile` (verify exact package
     name; may be split into `opennav-docking` + `opennav-docking-bt`).
  2. Add a `docking_server` Node to the Nav2 group in `njord.launch.py` and
     include `"docking_server"` in the `lifecycle_manager` `node_names`.
  3. Add a `docking_server:` block to `bringup/config/nav2_params.yaml` with:
     - `controller:` (graceful_controller params — works for forward-only USV)
     - `dock_plugins:` list of supported dock types
     - `docks:` static instances OR `dock_database` YAML path
  4. Decide on dock-pose source — options:
     - **Hardcoded GPS**: cheapest, fragile, fine for static known docks
     - **Vision-based**: AprilTag/ArUco detector publishing dock pose, or a
       YOLO class for the dock target with PnP for pose
  5. Custom BT XML that sequences `NavigateToPose` → `DockRobot` → mission
     continuation (default Nav2 trees don't include docking nodes).
  6. Sim verification before water: add a dock model to `basicWorld.sdf` and
     run a full nav-to-dock sequence end-to-end.

## Sensor Data Processing Tests

- [ ] **Verify `lidar_obstacle_node` output in sim**
  Launch with `use_sim:=true`, check `/obstacles/lidar` is published at ~15 Hz with `width > 0`.
  Also confirm `header.frame_id = "lidar"` and that range filtering (0.1–10 m) works correctly
  (objects at >10 m should not appear).

- [ ] **Verify `fusion_node` lidar passthrough (no YOLO)**
  With `enable_vision:=false`, `fusion_node` should echo all `/obstacles/lidar` points into
  `/obstacles/fused` with `frame_id = "base_link"`. Confirm: same point count, correct frame.
  TF lookup (`lidar → front_camera`) should succeed (check no `LookupException` in logs).

- [ ] **Verify `fusion_node` with YOLO active**
  With `enable_vision:=true`, place a visible object in Gazebo. Confirm `/yolo/detections`
  arrives, `/yolo/seg_mask` arrives, and `/obstacles/fused` combines both sources.
  YOLO-only detections (no lidar match) should appear at `DEFAULT_OBSTACLE_DISTANCE = 5.0 m`.

- [ ] **Verify EKF input rates**
  After `use_sim:=true` launch, check:
  - `ros2 topic hz /odom` → ~30 Hz (Gazebo OdometryPublisher)
  - `ros2 topic hz /imu_driver/imu_raw` → ~200 Hz (Gazebo IMU)
  - `ros2 topic hz /odometry/filtered` → ~30 Hz (EKF output)
  Low or missing rates indicate a broken bridge or plugin.

- [ ] **Verify costmap receives `/obstacles/fused`**
  After nav2 activates, echo `/local_costmap/costmap` and move a sim obstacle near the robot.
  Confirm the costmap inflates around the obstacle position reported by `/obstacles/fused`.

- [ ] **Verify sensor drivers start cleanly on hardware (no hardware attached)**
  `lidar_driver` should log "RPLIDAR not available... retrying" without crashing.
  `camera_driver` should log a degraded-mode warning without crashing.
  `imu_gps_driver` should wait for MAVROS without crashing.

## Perception / Fusion

- [ ] **Implement real late-fusion projection in `fusion_node`**
  `perception/perception/fusion_node.py` does not do actual camera projection.
  It estimates bearing from bbox centre pixel and places obstacles at a hardcoded
  5 m range. Replace with proper pipeline:
  1. Subscribe to raw `/points` (PointCloud2 from lidar) **in addition to** `/obstacles/lidar`
  2. Subscribe to `/camera/camera_info` for intrinsics matrix K
  3. Look up `camera_optical_link → lidar_link` TF at message time
  4. Project each 3D LIDAR point onto the image plane, check if it falls inside a
     YOLO segmentation bbox (or mask when available); label matching points semantically
  5. Fall back to clustered `/obstacles/lidar` for points outside any detection

- [ ] **Publish `CameraInfo` from `camera_driver`**
  `sensors/sensors/camera_driver.py` only publishes `/image_raw`. It must also
  publish `/camera/camera_info` (sensor_msgs/CameraInfo) for fusion projection.
  Calibrate the camera and store K, D in a YAML; load at startup.

- [ ] **Add in-memory object persistence to `fusion_node`**
  The node is stateless — the same buoy is re-fused every frame. Add a
  lightweight object map (dict of id → position + last_seen timestamp) with
  nearest-neighbour association (threshold ~2 m) and a configurable TTL
  (e.g. 8 s). Publish map state as a separate `/obstacles/tracked` topic.

## Control

- [ ] **Close the speed loop in `pid_controller`**
  `control/control/pid_controller.py` runs the speed PID open-loop (no feedback
  sensor). Provide speed feedback — options: use `/mavros/local_position/velocity_body`
  (ArduPilot EKF output), or `/odometry/filtered` from robot_localization.
  Subscribe to whichever is available and feed the measured linear speed as the
  process variable.

- [ ] **Verify RC channel mapping in `actuator_driver`**
  Channel indices (`CHAN_STEERING=0`, `CHAN_THROTTLE=2`) and the `RC_RANGE`
  scaling are placeholders. Confirm against the ArduPilot frame/channel
  assignment for the specific boat configuration (Rover skid-steer vs rudder+throttle).

## Navigation (Nav2)

- [ ] **Tune Nav2 controller for boat dynamics**
  `config/nav2_params.yaml` uses `RegulatedPurePursuitController`. For a USV
  with inertia and no skid-steering, evaluate switching to MPPI
  (`nav2_mppi_controller`) or tuning DWB with a diff-drive model that matches
  the boat's turning radius and maximum surge speed. At minimum, set
  `desired_linear_vel`, `lookahead_dist`, and `min_lookahead_dist` based on
  real on-water measurements.

## Infrastructure

- [ ] **Bind-mount config at runtime instead of baking it in the image**
  `config/` is `COPY`-ed into the image at build time. Field params (EKF
  covariances, Nav2 speeds, waypoints) change between tests. Mount
  `./config:/config:ro` in `compose.yaml` so tuning doesn't require a rebuild.
