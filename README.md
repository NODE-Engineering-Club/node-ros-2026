# Njord 2026

ROS 2 Jazzy autonomous surface vessel (USV) stack.

## Getting Started

**Prerequisites (one-time install):**
1. [VSCode](https://code.visualstudio.com/)
2. [Docker Desktop](https://www.docker.com/products/docker-desktop/) — Windows / macOS. On Linux, Docker Engine or Podman works.
   - Linux/Podman: set `"dev.containers.dockerPath": "podman"` in VSCode user settings.
3. VSCode extension: [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

**To start developing:**
1. Open this folder in VSCode
2. Click **Reopen in Container** when prompted (or `Ctrl+Shift+P` → *Dev Containers: Reopen in Container*)
3. First time takes ~5 minutes to build. After that it's instant.

The `postCreateCommand` automatically runs `colcon build --symlink-install` and `source install/setup.bash` is added to the shell, so everything is ready on first open.

> **Camera:** camera passthrough is optional. The dev container no longer requires `/dev/video0`, and Pi init only passes it through when present. If you don't have a webcam, the camera driver starts in degraded mode and the rest of the stack still works.

**Run the full stack:**
```bash
ros2 launch bringup njord.launch.py
```

**Run with selective subsystems disabled (e.g. no FCU, no Nav2):**
```bash
ros2 launch bringup njord.launch.py enable_mavros:=false enable_localization:=false enable_nav2:=false
```

**Rebuild after adding new files** (`--symlink-install` means code edits don't need a rebuild):
```bash
colcon build --symlink-install
source install/setup.bash
```

## Debugging

Run only the subsystems you care about by disabling everything else:

```bash
# Camera + vision only
ros2 launch bringup njord.launch.py \
  enable_mavros:=false \
  enable_localization:=false \
  enable_nav2:=false \
  enable_control:=false \
  enable_mission:=false \
  enable_perception:=false 
```

Then in a separate terminal:

```bash
# See what's publishing
ros2 topic list

# Stream detections
ros2 topic echo /yolo/detections

# Check frame rate
ros2 topic hz /yolo/detections
```

## Workspace layout

```
src/
├── sensors/      # camera_driver, lidar_driver, imu_gps_driver
├── perception/   # lidar_obstacle_node, fusion_node
├── control/      # nav_to_pid, pid_controller, actuator_driver
├── mission/      # mission_manager
├── vision/       # vision_node (YOLO ONNX inference)
└── bringup/      # launch/njord.launch.py + config/
models/           # ONNX weights (bind-mounted, gitignored)
```

## Architecture

```mermaid
flowchart TD
    subgraph Sensors
        L[lidar_driver] --> Scan[/scan/]
        C[camera_driver] --> Image[/image_raw/]
        I[imu_gps_driver]
    end

    subgraph Perception
        Y[vision_node] --> Det[/yolo/detections/]
        O[lidar_obstacle_node]
        F[fusion_node]
    end

    subgraph Localization
        RL[robot_localization]
        NT[navsat_transform]
    end

    subgraph Navigation[Navigation - Nav2]
        CM[costmap]
        PS[planner_server]
        CS[controller_server]
        BT[bt_navigator]
    end

    subgraph Control
        N2P[nav_to_pid]
        PID[pid_controller]
        ACT[actuator_driver]
    end

    subgraph Mission
        MM[mission_manager]
    end

    C --> Y
    L --> O
    Y --> F
    O --> F
    F --> CM
    I --> RL
    RL --> NT
    NT --> CM
    CM --> PS
    PS --> BT
    BT --> CS
    CS --> N2P
    N2P --> PID
    PID --> ACT
    MM --> BT
```

## Production deploy

```bash
podman compose build
podman compose up -d
podman compose logs -f
```
