#!/bin/bash
echo "Building ROS workspace..."
colcon build --symlink-install

echo "Sourcing ROS workspace..."
source /opt/ros/humble/setup.bash
source install/setup.bash