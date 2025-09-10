from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            arguments=['compressed', 'raw'],
            remappings=[
                ('compressed_in', '/drone1/image_raw/compressed'),
                ('raw_out', '/drone1/image_raw'),
            ]
        ),
        Node(
            package='orbslam3',
            executable='mono',
            name='orbslam3_tello',
            output='screen',
            arguments=[
                '/home/lucijan/orbslam3_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt',
                '/home/lucijan/orbslam3_ws/src/orbslam3_ros2/config/monocular/tello_camera.yaml',
#                '/home/lucijan/orbslam3_ws/src/orbslam3_ros2/config/monocular/tello_camera_real.yaml'
            ],
            remappings=[
                ('/camera', '/drone1/image_raw'),
#                ('/camera', '/image_raw'),
            ]
        )
    ])
