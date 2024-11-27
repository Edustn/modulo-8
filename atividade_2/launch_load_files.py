from launch import LaunchDescription
from launch.actions import OpaqueFunction
import subprocess

# Função para executar o subprocess
def run_teleop(context, *args, **kwargs):
    subprocess.Popen(['ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'], stdin=None)
    return []

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=run_teleop)
    ])
