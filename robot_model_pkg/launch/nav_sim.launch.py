import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name='robot_model_pkg' #<--- CHANGE ME
    map_file=os.path.join(get_package_share_directory(package_name),'maps','my_map.yaml')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name),'launch','sim.launch.py')]),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name),'launch','base','navigation.launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml'),
            'map': map_file,
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    return LaunchDescription([
        sim,
        nav2
    ])