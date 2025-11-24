from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_1',
            output='screen',
            parameters=[{
                'camera_topic': 'camera/back/rgb',
                'camera_type': 'USBCam',
                'camera_id': 0,
                'message_text': 'Front Camera Stream',
                'camera_serial': ''
            }]
        )
    ])