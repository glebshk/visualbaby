from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_detect',
            executable='detect',
            name='aruco_detection_bottom',
            output='screen',
            parameters=[{
                'listen_on_topic':'camera',
                'camera_type': 'USB',
                'camera_serial': "640480"
            }
            ]
                  
        )
        #Node(
        #    package='camera_stream',
        #    executable='stream',
        #    name='rtsp_stream',
        #    output='screen'
     #)
    ])
