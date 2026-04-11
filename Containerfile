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
    python3-venv \
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
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# --- Dev target: workspace is bind-mounted, used by devcontainer ---------------
FROM base AS dev
ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
WORKDIR /workspace

# --- Prod target: code baked in, used by compose.yaml -------------------------
FROM base AS prod

# GeographicLib datasets required by MAVROS GPS plugins
RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

COPY pyproject.toml  /app/
COPY sensors/        /app/sensors/
COPY perception/     /app/perception/
COPY control/        /app/control/
COPY mission/        /app/mission/
COPY vision_detector/ /app/vision_detector/
COPY config/         /config/
COPY launch/         /launch/

RUN python3 -m venv /opt/venv --system-site-packages \
    && /opt/venv/bin/pip install --no-cache-dir /app

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV PATH="/opt/venv/bin:$PATH"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/launch/njord.launch.py"]
