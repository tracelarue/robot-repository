import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robot_model_pkg'
    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config','gazebo_params.yaml'
    )
    world_file_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds','downstairs_combined.world'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path, 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Launch robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch','base','rsp.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Delay controller spawning to ensure robot is loaded first
    diff_drive_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont"],
                output="screen"
            )
        ]
    )

    arm_group_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_group_controller"],
                output="screen"
            )
        ]
    )

    hand_group_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["hand_group_controller"],
                output="screen"
            )
        ]
    )

    joint_broadcaster = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad"],
                output="screen"
            )
        ]
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config','twist_mux.yaml'
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_entity,
        joint_broadcaster,
        diff_drive_controller,
        arm_group_controller,
        hand_group_controller, 
        twist_mux,
    ])