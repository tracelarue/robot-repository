# Robot Arm Package Launch Configuration

## Overview
This package has been cleaned up to use centralized configuration for better maintainability and consistency.

## Changes Made

### 1. Centralized Configuration (`launch/common_config.py`)
- **Package Name**: Centralized `PACKAGE_NAME = 'robot_arm'` 
- **Default Parameters**: 
  - `DEFAULT_USE_SIM_TIME = 'false'`
  - `DEFAULT_USE_ROS2_CONTROL = 'true'`
  - `DEFAULT_USE_FAKE_HARDWARE = 'false'`
- **File Paths**: All common file paths are now centralized:
  - `URDF_FILE`
  - `RVIZ_CONFIG`
  - `GAZEBO_PARAMS`
  - `CONTROLLER_CONFIG`
  - `TWIST_MUX_CONFIG`
  - `WORLD_FILE`
- **Log Levels**: Centralized log level configuration

### 2. Updated Launch Files

#### `launch/base/rsp.launch.py`
- Uses centralized configuration from `common_config.py`
- Added `use_fake_hardware` parameter
- Fixed parameter name from `use_sim` to `use_sim_time`
- Cleaner xacro command invocation
- Better formatted launch arguments

#### `launch/robot.launch.py`
- Fixed incorrect package name (`robot_model_pkg` → `robot_arm`)
- Uses centralized configuration
- Cleaner file path construction

#### `launch/sim.launch.py`
- Uses centralized configuration
- Cleaner file path construction

## Usage

### Standard Parameters
All launch files now consistently support:
- `use_sim_time`: Use simulation time (default: false)
- `use_ros2_control`: Enable ros2_control (default: true)
- `use_fake_hardware`: Use fake hardware for testing (default: false)

### Example Usage
```bash
# Launch with simulation time
ros2 launch robot_arm rsp.launch.py use_sim_time:=true

# Launch with fake hardware
ros2 launch robot_arm rsp.launch.py use_fake_hardware:=true

# Launch simulation
ros2 launch robot_arm sim.launch.py
```

## Benefits

1. **Consistency**: All launch files use the same parameter names and defaults
2. **Maintainability**: Changes to package name or file paths only need to be made in one place
3. **Readability**: Clear separation of configuration from launch logic
4. **Extensibility**: Easy to add new common parameters or file paths

## Future Improvements

Consider adding:
- Environment-specific configurations (dev, staging, prod)
- Robot-specific parameter sets
- Validation for parameter values
