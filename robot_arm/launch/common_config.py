"""
Common configuration parameters for robot_arm package launch files.
This centralizes package-wide parameters to improve consistency and maintainability.
"""

# Package identification
PACKAGE_NAME = 'robot_arm'

# Default parameter values
DEFAULT_USE_SIM_TIME = 'false'
DEFAULT_USE_ROS2_CONTROL = 'true'
DEFAULT_USE_FAKE_HARDWARE = 'false'

# Common file paths
URDF_FILE = 'urdf/robot.urdf.xacro'
RVIZ_CONFIG = '/home/trace/robot/sim.rviz'
GAZEBO_PARAMS = 'config/gazebo_params.yaml'
CONTROLLER_CONFIG = 'config/gazebo_controller_manager.yaml'
TWIST_MUX_CONFIG = 'config/twist_mux.yaml'
WORLD_FILE = 'worlds/downstairs_combined.world'

# Log levels
RVIZ_LOG_LEVEL = 'WARN'
ROSCONSOLE_MIN_SEVERITY = 'WARN'
