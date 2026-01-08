#!/bin/bash
echo "Building ROS workspace..."
colcon build --symlink-install

echo "Sourcing ROS workspace..."
source install/setup.bash