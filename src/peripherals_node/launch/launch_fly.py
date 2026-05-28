from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='peripherals_node',
            executable='peripherals_node',
            name='peripherals_node',
            output='screen',
            parameters=[{
                'env': 'drone'
            }]
        )]) 