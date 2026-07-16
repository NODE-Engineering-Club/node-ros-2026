#!/bin/bash
pip install --break-system-packages simple-pid
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
