FROM docker.io/ros:jazzy-ros-base AS base

RUN apt-get update && apt-get install -y --no-install-recommends \
    # MAVROS2 + MAVLink
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-mavros-msgs \
    ros-jazzy-geographic-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    # Sensors
    python3-opencv \
    v4l-utils \
    ros-jazzy-cv-bridge \
    # Vision / Perception
    libgl1 \
    ros-jazzy-vision-msgs \
    # Localization
    ros-jazzy-robot-localization \
    # Navigation
    ros-jazzy-nav2-bringup \
    # Control / Mission
    ros-jazzy-nav2-msgs \
    # Telemetry bridge
    # ros-jazzy-rosbridge-suite \
    # ros-jazzy-web-video-server \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-ros-gz-bridge \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout root

# --- Dev target: workspace is bind-mounted, used by devcontainer ---------------
FROM base AS dev
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' >> /etc/bash.bashrc
WORKDIR /workspace

# --- Prod target: code baked in, built with colcon ----------------------------
FROM base AS prod

# GeographicLib datasets required by MAVROS GPS plugins
RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# pip-only deps (no rosdep keys exist for these)
RUN pip install --break-system-packages --no-cache-dir \
     "numpy<2" \
     onnxruntime \
     opencv-python-headless \
     rplidar-roboticia

COPY src/ /ros2_ws/src/

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
        --base-paths /ros2_ws/src \
        --install-base /opt/njord \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "bringup", "njord.launch.py"]
