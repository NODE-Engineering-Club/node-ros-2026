# Node

## Getting Started

**Prerequisites (one-time install):**
1. [VSCode](https://code.visualstudio.com/)
2. [Docker Desktop](https://www.docker.com/products/docker-desktop/) — Windows / macOS. On Linux, Docker Engine or Podman works too.
3. VSCode extension: [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

**To start developing:**
1. Open this folder in VSCode
2. Click **Reopen in Container** when prompted (or `Ctrl+Shift+P` → *Dev Containers: Reopen in Container*)
3. First time takes ~5 minutes to build. After that it's instant.

That's it. You'll have a full ROS 2 environment with linting, autocomplete, and all entry points available. Open the integrated terminal and run the stack:

```bash
ros2 launch /workspace/launch/njord.launch.py
```

## Architecture

```mermaid
flowchart TD
    subgraph Sensors
        L[lidar_driver] --> Scan[/scan or /points/]
        C[camera_driver] --> Image[/image_raw/]
        I[imu_gps_driver]
    end

    subgraph Perception
        Y[yolo_seg_node] --> Det[/yolo/masks + classes/]
        O[lidar_obstacle_node]
        F[fusion_node<br/>← PROJECTS segmented buoys/boats onto LIDAR]
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
        N2P[navigation_to_pid]
        PID[pid_controller_node]
        ACT[actuator_driver]
    end

    subgraph Mission
        MM[mission_manager]
    end

    C --> Y
    L --> O
    L -.-> F
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
