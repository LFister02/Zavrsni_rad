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
            period=3.0,  # čekaj 3 sekunde da se RViz pokrene
            actions=[

                Node(
                    package='a_star_test',
                    executable='a_star',	
                    name='a_star_nr_test',
                    output = 'screen',
                    parameters = [
                        {"file_name" : "ime_CSV_datoteke"},   # Ime datoteke u kojoj se spremaju podaci testiranja
                        {"grid_size_x" : 100},   # Definiranje dimenzija 3D kvadratne mreže
                        {"grid_size_y" : 100},
                        {"grid_size_z" : 6},
                        {"num_obstacles" : 400}, # Definiranje broja prepreka
                        {"max_obstacle_height" : 6},    # Definiranje maksimalne visine prepreka
                        {"wanted_iterations" : 500},     # Definiranje broja iteracija
                        {"heuristic_type" : "euclidean"},   # Definiranje tipa heuristike - Mogući odabir: [manhattan | euclidean]
                        {"kretanje": "dijagonalno"},    # Definiranje načona kretanja - Moguči odabir: [jednostavno | dijagonalno], gdje je jednostavno = 6 smjerova, a dijagonalno = 26 smjerova
                        {"heuristic_gore" : 1.0},   # Definiranje kazne
                        {"heuristic_isto" : 1.0},
                        {"heuristic_dolje" : 1.0},
                        {"save_results_csv" : False},   # Postavljanje snimanja podataka [True | False]
                        {"work_type" : "point_click"}    # Postavljanje načina rada algoritma s obzirom na pozicije starta i cilja
                                                        # Načini rada: [non_random | random | point_click | penalty]
                    ]
                ),
            ]
        )
])
