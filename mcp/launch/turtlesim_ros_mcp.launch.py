
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    turtlesim_node = Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        )
    
        # Launch rosbridge server using IncludeLaunchDescription
    rosbridge_server = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    claude_desktop = ExecuteProcess(
        cmd=['claude-desktop'],
        output='screen',
    )


    return LaunchDescription([
        turtlesim_node,
        rosbridge_server,
        claude_desktop
    ])