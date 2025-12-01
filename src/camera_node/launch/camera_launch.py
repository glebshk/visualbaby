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
                'camera_topic': 'camera/front/rgb',
                'camera_type': 'USBCam',
                'camera_id': 0,
                'message_text': 'Front Camera Stream',
                'camera_serial': ''
            }]
        ),
        #Node(
         #   package='camera_node',
          #  executable='camera_node',
           # name='camera_real',
           # output='screen',
           # parameters=[{
           #     'camera_topic': 'camera/bottom/rgb',
           #     'camera_type': 'Realsense',
           #     'camera_id': 0,
           #     'message_text': 'Bottom Cam',
           #     'camera_serial': 918512071813
           # }]
        #),
        
        Node(
            package='camera_stream',
            executable='stream',
            name='rtsp_stream',
            output='screen'
     )
    ])