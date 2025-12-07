from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_usb',
            output='screen',
            parameters=[{
                'camera_topic': 'camera/bottom/rgb',
                'camera_type': 'USBCam',
                'camera_id': 0,
                'message_text': 'Bottom Camera Stream'
            }]
        ),
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_real',
            output='screen',
            parameters=[{
                'camera_topic': 'camera/front/rgb',
                'camera_type': 'Realsense',
                'message_text': 'Front Camera',
                'camera_serial': "918512073048",
                'camera_rotation': 180
            }]
        ),
        
        Node(
            package='aruco_detect',
            executable='detect',
            name='aruco_detection_bottom',
            output='screen',
            parameters=[{
                'listen_on_topic':'camera/bottom/rgb',
                'camera_type': 'USB_CAM'
            }
            ]
                    
        ),
        Node(
            package='aruco_detect',
            executable='detect',
            name='aruco_detection_front',
            output='screen',
            parameters=[{
                'listen_on_topic':'camera/front/rgb',
                'camera_type': 'Realsense',
                'camera_serial': "918512073048"
            }
            ]
                    
        ),
        Node(
            package='camera_stream',
            executable='stream',
            name='rtsp_stream',
            output='screen'
     )
    ])
