# Njord 2026 — Outstanding Gaps

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

## Transforms (TF)

- [x] **TF tree implemented and verified**
  Fixed three frame-name mismatches that were silently breaking the entire stack:
  - Renamed URDF root link `hull` → `base_link` (EKF and Nav2 both expect `base_link`)
  - Renamed URDF links `Lidar` → `lidar`, `Lidar_joint` → `lidar_mount` (to match `lidar_driver` `frame_id`)
  - Added `frame_id` parameter to `camera_driver` (default `front_camera`); passed explicitly in launch
  - Added explicit `lidar_frame`/`camera_frame` params to `fusion_node` in launch
  `robot_state_publisher` now broadcasts the complete static sensor tree from the URDF.
  Verified with `ros2 run tf2_tools view_frames` — full tree present, all frames correct.

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

- [ ] **Set competition waypoints in `mission_manager`**
  `mission/mission/mission_manager.py` has placeholder coordinates
  (Trondheim test area). Replace with actual Njord 2026 competition waypoints
  before deployment.

## Sensors

- [x] **Camera device path configurable**
  `camera_driver` accepts a `device` ROS parameter (default `/dev/video0`) and a
  `frame_id` parameter (default `front_camera`). Override via `camera_device` launch
  argument, e.g. `ros2 launch bringup njord.launch.py camera_device:=/dev/video1`.

## Infrastructure

- [ ] **Add a hardware-in-the-loop smoke test**
  No integration test exists. Write a minimal test that:
  1. Launches the full stack with `ros2 launch /launch/njord.launch.py`
  2. Asserts that `/obstacles/fused`, `/odometry/filtered`, and `/cmd_vel` are
     publishing within 10 s of startup
  Run this as a CI check before every deploy.

- [ ] **Bind-mount config at runtime instead of baking it in the image**
  `config/` is `COPY`-ed into the image at build time. Field params (EKF
  covariances, Nav2 speeds, waypoints) change between tests. Mount
  `./config:/config:ro` in `compose.yaml` so tuning doesn't require a rebuild.
