# Voice-Controlled Drink Retrieval Robot - Development Plan

## Project Overview
Building a ROS2 Humble robot that responds to voice commands to autonomously navigate, locate, and retrieve drinks using Google Gemini Live API, Nav2, and MoveIt2.

**Current Status:**
- ✅ Gemini voice interface working (sim + real robot)
- ✅ Navigation2 working (sim + real robot) 
- ✅ MoveIt2 working in simulation
- ✅ Object 3D location detection (camera + depth)
- ❌ Pick and place manipulation 
- ❌ Real robot arm integration
- ❌ Complete workflow integration

## Phase 1: Complete Pick and Place Implementation (2-3 weeks)

### 1.1 Implement MoveIt2 Pick and Place Service (Week 1)
**Priority: HIGH**

Create a dedicated ROS2 service node for pick and place operations:

```bash
# Create new package
cd /home/trace/robot/src
ros2 pkg create --build-type ament_cmake pick_place_service --dependencies rclcpp moveit_core moveit_ros_planning_interface geometry_msgs std_srvs
```

**Key Components to Implement:**
- **Pick Service**: `/pick_object` 
  - Input: 3D position (Point), object dimensions (optional)
  - Output: success/failure, error messages
- **Place Service**: `/place_object`
  - Input: 3D position for placement
  - Output: success/failure, error messages

**Implementation approach** (based on MoveIt2 documentation):
```cpp
// Use MoveIt Task Constructor (MTC) for robust pick/place
// Key stages:
// 1. Current state → Open hand → Approach → Grasp → Lift → Place → Return
```

**Reference files to study:**
- `/home/trace/robot/src/robot_arm/launch/moveit_launch_ex.py` (your existing MoveIt config)
- MoveIt2 pick/place examples from Context7 docs

### 1.2 Integrate Depth Perception with Pick Planning (Week 1-2)
**Priority: HIGH**

Enhance the existing depth detection in Gemini to provide properly formatted pick poses:

**Current depth detection location:**
- `/home/trace/robot/src/gemini/gemini/MultiModal_*.py` files have depth detection
- Need to create service interface between Gemini and pick service

**Implementation:**
- Create `/get_object_pose` service 
- Input: object description/name
- Output: geometry_msgs/PoseStamped with 6DOF pose for grasping
- Integrate with existing vision system in Gemini

### 1.3 Test Pick and Place in Simulation (Week 2)
**Priority: HIGH**

**Test sequence:**
1. Launch simulation with arm: `ros2 launch robot_arm moveit_launch_ex.py`
2. Place test objects in Gazebo/RViz
3. Test pick service: `ros2 service call /pick_object ...`
4. Verify gripper operation and object attachment
5. Test place service and object detachment

## Phase 2: Real Robot Hardware Integration (2-3 weeks)

### 2.1 Implement Real Arm Hardware Interface (Week 3-4)
**Priority: HIGH**

**Based on ros2_control documentation, you need:**

```cpp
// Create hardware interface class inheriting from hardware_interface::SystemInterface
class RealArmHardware : public hardware_interface::SystemInterface {
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  // ... other lifecycle methods
};
```

**Steps:**
1. **Identify your arm hardware:**
   - What specific arm/gripper do you have?
   - Communication protocol (USB, Serial, Ethernet, CAN)?
   - Existing drivers/SDKs?

2. **Create hardware interface:**
   - Copy pattern from `/home/trace/robot/src/diffdrive_arduino/` 
   - Implement actual hardware communication in `read()`/`write()` methods
   - Export joint position/velocity command and state interfaces

3. **Update URDF:**
   - Modify `/home/trace/robot/src/robot_arm/` URDF to use real hardware
   - Change `<hardware>` section from mock to your real interface

### 2.2 Real Robot Integration Testing (Week 4-5)
**Priority: MEDIUM**

**Test progression:**
1. Hardware interface loads: `ros2 control list_hardware_interfaces`
2. Controllers start: `ros2 control list_controllers`
3. Joint commands work: Test individual joint movements
4. MoveIt2 planning works with real hardware
5. Gripper open/close functions
6. Full pick/place sequence

### 2.3 Depth Camera Integration on Real Robot (Week 4)
**Priority: MEDIUM**

**Current status:** Gemini can get depth in sim but not real robot

**Implementation:**
- Check depth camera drivers are installed
- Verify camera topics: `ros2 topic list | grep depth`
- Debug Gemini depth detection on real robot
- Ensure camera calibration is correct

## Phase 3: Complete Workflow Integration (1-2 weeks)

### 3.1 Create Gemini Pick and Place Tool (Week 5-6)
**Priority: HIGH**

Add new function calling tool to Gemini API:

```python
# In MultiModal_*.py files
pick_and_place_tool = {
    "name": "pick_and_place_object",
    "description": "Pick up an object and place it at a location",
    "parameters": {
        "type": "object",
        "properties": {
            "object_description": {"type": "string"},
            "place_location": {"type": "string"},  # "user", "table", etc.
        }
    }
}
```

**Integration flow:**
1. User: "Get me a coke from the fridge"
2. Gemini navigates to fridge
3. Gemini identifies coke location with vision
4. Gemini calls pick service with 3D coordinates
5. Gemini navigates back to user
6. Gemini places object near user

### 3.2 Complete End-to-End Testing (Week 6)
**Priority: HIGH**

**Test scenarios:**
1. **Basic retrieval:** "Get the red cup from the table"
2. **Fridge scenario:** "Bring me a water bottle from the fridge"
3. **Error handling:** Object not found, navigation failures, pick failures
4. **Multi-object:** "Bring me two drinks"

## Phase 4: Optimization and Polish (1-2 weeks)

### 4.1 Performance Optimization (Week 7)
**Priority: MEDIUM**

- **Navigation optimization:** Tune Nav2 parameters for your environment
- **Manipulation optimization:** Optimize MoveIt2 planning time and success rate
- **Vision optimization:** Improve object detection reliability

### 4.2 Error Handling and Recovery (Week 7-8)
**Priority: MEDIUM**

- **Robust pick/place:** Retry mechanisms, approach angle variations
- **Navigation recovery:** Handle blocked paths, map updates
- **Voice interaction:** Clear error communication to user

### 4.3 User Experience Improvements (Week 8)
**Priority: LOW**

- **Status updates:** "I'm going to the fridge now", "I found the drink", etc.
- **Confirmation:** "I see a red can, is this what you want?"
- **Multiple options:** Handle when multiple matching objects are found

## Technical Architecture Recommendations

### 1. Service-Based Architecture
Use ROS2 services for clean separation:
```
Gemini Node ←→ Navigation Service ←→ Nav2
            ←→ Vision Service ←→ Camera/Depth
            ←→ Manipulation Service ←→ MoveIt2
```

### 2. State Management
Implement a simple state machine:
- IDLE → NAVIGATING → SEARCHING → PICKING → RETURNING → PLACING → IDLE

### 3. Configuration Management
Create centralized config files:
```yaml
# robot_config.yaml
objects:
  drinks:
    - {name: "coke", size: [0.06, 0.06, 0.12], grasp_height: 0.06}
    - {name: "water", size: [0.05, 0.05, 0.25], grasp_height: 0.12}
locations:
  fridge: {x: 2.5, y: -5.94}
  table: {x: 0.0, y: 0.0}
```

## Development Workflow Suggestions

### 1. Development Environment
```bash
# Always source all workspaces
source /home/trace/robot/install/setup.bash
source /home/trace/turtlebot3_ws/install/setup.bash  
source /home/trace/moveit_ws/install/setup.bash

# Build incrementally
colcon build --packages-select pick_place_service
```

### 2. Testing Strategy
- **Unit tests:** Test each service independently
- **Integration tests:** Test service combinations
- **End-to-end tests:** Full voice command workflow
- **Simulation first:** Always test in simulation before real robot

### 3. Debug Tools
```bash
# Monitor topics
ros2 topic echo /joint_states
ros2 topic echo /camera/depth/image_raw

# Check services
ros2 service list | grep pick
ros2 service call /pick_object pick_place_service/Pick "{position: {x: 0.5, y: 0.0, z: 0.1}}"

# Visualize in RViz
ros2 run rviz2 rviz2 -d /home/trace/robot/sim.rviz
```

## Risk Mitigation

### High-Risk Items:
1. **Real hardware integration complexity** → Start with simple movements, incremental testing
2. **Pick and place reliability** → Multiple grasp attempts, different approach angles
3. **Vision-manipulation coordination** → Robust coordinate frame transformations

### Medium-Risk Items:
1. **Depth camera calibration** → Use well-tested calibration procedures
2. **Navigation in cluttered environments** → Good map building, safety margins

## Success Metrics

### Minimum Viable Product (MVP):
- [ ] Robot responds to "get me a drink" command
- [ ] Navigates to known location (table/fridge)
- [ ] Identifies and picks up a simple object (cup/bottle)
- [ ] Returns to user and places object nearby
- [ ] Success rate > 70% in controlled environment

### Stretch Goals:
- [ ] Handles multiple object types and sizes
- [ ] Works in dynamic environments
- [ ] Provides natural voice feedback
- [ ] Success rate > 90%
- [ ] Works with objects in fridge (more complex manipulation)

## Next Immediate Steps (This Week):

1. **Create pick_place_service package** (Day 1-2)
2. **Implement basic pick service using MoveIt2** (Day 2-3)
3. **Test pick service in simulation** (Day 4-5)
4. **Start hardware interface for real arm** (Day 5-7)

## Resources and Documentation:
- **MoveIt2 Pick/Place:** Context7 docs from `/context7/moveit_picknik_ai-humble`
- **ros2_control:** Context7 docs from `/context7/control_ros_org-humble-index.html`
- **Nav2:** Context7 docs from `/ros-navigation/docs.nav2.org`
- **Your existing code:** Study `/home/trace/robot/src/` packages

This plan focuses on getting a working system quickly while maintaining good software engineering practices. The key is to iterate rapidly in simulation before moving to real hardware.
