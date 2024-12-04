from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Incluir o arquivo de lançamento do Gazebo com o TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('turtlebot3_gazebo'), '/launch/turtlebot3_world.launch.py']
            ),
            launch_arguments={}.items()
        ),
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            name='teleop',
            prefix='gnome-terminal --',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('turtlebot3_cartographer'), '/launch/cartographer.launch.py']
            ),
            launch_arguments={}.items()
        ),
    ])
