FROM docker.io/ros:jazzy-ros-base

ARG ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    # MAVROS2 + MAVLink
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    # Sensors
    python3-opencv \
    python3-venv \
    v4l-utils \
    ros-${ROS_DISTRO}-cv-bridge \
    # Vision / Perception
    libgl1 \
    ros-${ROS_DISTRO}-vision-msgs \
    # Localization
    ros-${ROS_DISTRO}-robot-localization \
    # Navigation
    ros-${ROS_DISTRO}-nav2-bringup \
    # Control / Mission
    ros-${ROS_DISTRO}-nav2-msgs \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

# GeographicLib datasets required by MAVROS GPS plugins
RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

COPY sensors/       /sensors/
COPY perception/    /perception/
COPY control/       /control/
COPY mission/       /mission/
COPY vision_detector/ /vision_detector/
COPY config/        /config/
COPY launch/        /launch/

# Single venv inherits ROS Python bindings via --system-site-packages.
# onnxruntime replaces the full ultralytics/torch stack for inference.
RUN python3 -m venv /opt/venv --system-site-packages \
    && /opt/venv/bin/pip install --no-cache-dir \
        onnxruntime \
        opencv-python-headless \
        numpy \
        /sensors \
        /perception \
        /control \
        /mission \
        /vision_detector

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV ROS_DISTRO=${ROS_DISTRO}
ENV PATH="/opt/venv/bin:$PATH"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/launch/njord.launch.py"]
