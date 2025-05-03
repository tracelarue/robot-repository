import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name='robot_model_pkg' #<--- CHANGE ME

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','robot.launch.py')]), 
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name),'launch','base','navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml'),
        }.items()
    )  


    return LaunchDescription([
        robot,
        nav2
    ])