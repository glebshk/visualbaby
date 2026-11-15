#!/bin/bash
# Install the aufdeckung core and build the ROS 2 workspace (run inside the dev container).
set -e
echo "Installing aufdeckung core..."
python3 -m pip install -e "aufdeckung[viz]"

echo "Building ROS workspace..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "Sourcing ROS workspace..."
source install/setup.bash
