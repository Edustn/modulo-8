from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mapa_yaml = '/home/edu-bat/Documentos/GitHub/modulo-8/atividade_2_slam/meu_workspace/launch.yaml'

    return LaunchDescription([
        IncludeLaunchDescription(   
            PythonLaunchDescriptionSource(
                [get_package_share_directory('turtlebot3_navigation2'), '/launch/navigation2.launch.py']
            ),  
            launch_arguments={
                'use_sim_time': 'True',
                'map': mapa_yaml
            }.items()
        ),

        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('turtlebot3_gazebo'), '/launch/turtlebot3_world.launch.py']
            ),
            launch_arguments={}.items()
        ),


    ])
