from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_to_ros2',
            executable='px4',
            name='ros_to_px4',
            output='screen',
            parameters=[{
                'envirement':'drone'
            }
            ]
                  
        )
    ])
