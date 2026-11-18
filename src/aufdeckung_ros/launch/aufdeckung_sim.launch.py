"""Launch the PX4 bridge, the Gazebo<->ROS bridge and the aufdeckung mission node (simulation)."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    repo = os.environ.get("AUFDECKUNG_REPO", os.getcwd())
    args = [
        DeclareLaunchArgument("home_x", default_value="0.0"),
        DeclareLaunchArgument("home_y", default_value="-18.0"),
        DeclareLaunchArgument("prior_scenario", default_value=""),
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("bridge_config",
                              default_value=os.path.join(repo, "simulation", "gz-bridge.yaml")),
    ]
    return LaunchDescription([
        *args,
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": LaunchConfiguration("bridge_config")}]),
        Node(package="px4_to_ros2", executable="px4", output="screen",
             parameters=[{"envirement": "simulator"}]),
        Node(package="aufdeckung_ros", executable="mission", output="screen",
             # Explicit types: launch arguments are otherwise parsed as YAML, which turns an empty
             # string into None and "3" into an int; either makes the node exit at startup.
             parameters=[{
                 "home_x": ParameterValue(LaunchConfiguration("home_x"), value_type=float),
                 "home_y": ParameterValue(LaunchConfiguration("home_y"), value_type=float),
                 "prior_scenario": ParameterValue(LaunchConfiguration("prior_scenario"), value_type=str),
                 "config_file": ParameterValue(LaunchConfiguration("config_file"), value_type=str),
                 "lidar_topic": "/lidar/points",
             }]),
    ])
