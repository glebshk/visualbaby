#!/bin/bash
# Stops everything launch-sim-aufdeckung.sh started: tmux session, Gazebo, PX4 and the ROS nodes.
# Safe to run when nothing is running.
tmux kill-session -t aufdeckung-sim 2>/dev/null && echo "tmux session closed"
pkill -f "gz sim" 2>/dev/null && echo "Gazebo stopped"
pkill -x px4 2>/dev/null && echo "PX4 stopped"
pkill -f "lib/aufdeckung_ros/mission|lib/px4_to_ros2/px4|ros_gz_bridge/parameter_bridge|ros2 launch aufdeckung_ros" 2>/dev/null && echo "ROS nodes stopped"
sleep 1
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -x px4 2>/dev/null
echo "simulation stopped"
