import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable



def generate_launch_description():
    pkg_name = 'wilson'
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Set RViz debug level environment variable to reduce terminal output
    set_rviz_log_level = SetEnvironmentVariable(
        name='ROSCONSOLE_MIN_SEVERITY',
        value='WARN'
    )

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    xacro_file = os.path.join(pkg_path,'urdf','wilson.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_sim_time:=', use_sim_time])
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params],
    )
    
    # Re-enable RViz2 node but with reduced debug output
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/robot/robot.rviz', '--ros-args', '--log-level', 'WARN'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Launch!
    return LaunchDescription([
        set_rviz_log_level,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher,
        node_rviz2
    ])
