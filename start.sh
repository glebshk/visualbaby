#!/bin/bash
set -e

cd /workspaces/IMAV-videostream/src

colcon build --symlink-install

# ROS2 Se
source install/setup.bash

ros2 launch camera_node camera_launch.py