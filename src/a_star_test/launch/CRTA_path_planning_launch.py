import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([

        Node(
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', '/home/lucijan/zavrsni_ws/src/a_star_test/config/zavrsni.rviz'],
            
        ),

        TimerAction(
            period=4.0,  # čekaj 4 sekunde da se RViz pokrene
            actions=[

                Node(
                    package='a_star_test',
                    executable='tello_map_real',	
                    name='tello_map_real',
                    output = 'screen',
                                                  
                ),

                Node(
                    package='a_star_test',
                    executable='tello_follower',	
                    name='tello_follower',
                    output = 'screen',
                ),
            ]
        )
])
