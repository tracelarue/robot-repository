import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    package_name = 'robot_model_pkg'

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'sim.launch.py')
        )
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'base', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml'),
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    return LaunchDescription([
        sim,
        nav2
    ])
