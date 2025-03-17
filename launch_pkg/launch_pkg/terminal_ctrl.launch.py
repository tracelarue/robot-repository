from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch the terminal control and serial output nodes."""

    ld= LaunchDescription ()
    
    # Define the terminal control node
    terminal_ctrl_node = Node(
        package='terminal_ctrl_pkg',
        executable='terminal_ctrl_node',
        name='terminal_ctrl_node',
        output='screen',
        emulate_tty=True,
    )
    
    # Define the serial output node

    serial_out_node = Node(
        package='serial_coms',  # Replace with actual package name if different
        executable='serial_out_node',
        name='serial_out_node',
        output='screen',
    )
    
    ld.add_action( terminal_ctrl_node )
    ld.add_action( serial_out_node )

    return ld