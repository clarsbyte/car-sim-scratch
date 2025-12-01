# ROS2 Differential Drive Robot - Controller Fix Documentation

## Problem Summary

The robot's controller_manager was not starting, resulting in the following error:
```
[WARN] Could not contact service /controller_manager/list_hardware_interfaces
[INFO] waiting for service /controller_manager/list_hardware_interfaces to become available...
```

## Root Causes Identified

### 1. Xacro File Not Being Processed (Critical)
**File**: `src/sim/launch/gazebo.launch.py`

**Problem**: The launch file was reading the `.xacro` file as plain text instead of processing it:
```python
# BEFORE (BROKEN)
with open(urdf_file, 'r') as infp:
    robot_desc = infp.read()
```

**Impact**: The `ros2_control.xacro` include was never expanded, so Gazebo never loaded the ros2_control plugin, and the controller_manager was never started.

**Fix**: Process the xacro file to expand all includes and macros:
```python
# AFTER (WORKING)
import xacro
robot_desc = xacro.process_file(urdf_file).toxml()
```

### 2. Joint Name Mismatch
**File**: `src/sim/urdf/ros2_control.xacro`

**Problem**: The ros2_control configuration referenced joint names that didn't match the actual URDF:
- ros2_control used: `left_front_wheel_joint`, `right_front_wheel_joint`, etc.
- URDF defined: `body_to_left_front_wheel`, `body_to_right_front_wheel`, etc.

**Fix**: Updated all joint names in `ros2_control.xacro` to match the URDF joints:
```xml
<joint name="body_to_left_front_wheel">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
</joint>
<!-- Same for all 4 wheel joints -->
```

### 3. Controller Configuration Mismatch
**File**: `src/sim/config/controllers.yaml`

**Problem**:
1. Controller wheel names didn't match the actual joint names
2. Integer values used instead of floats causing: `expected [double] got [integer]` error

**Fix**:
1. Updated wheel names in the diff_cont configuration
2. Changed all rate values to explicit floats:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 30.0  # Was 30 (integer)

diff_cont:
  ros__parameters:
    publish_rate: 50.0  # Was 50 (integer)
    left_wheel_names: ['body_to_left_back_wheel', 'body_to_left_front_wheel']
    right_wheel_names: ['body_to_right_back_wheel', 'body_to_right_front_wheel']
```

### 4. Missing Controller Spawners
**File**: `src/sim/launch/gazebo.launch.py`

**Problem**: The launch file didn't include nodes to spawn the controllers.

**Fix**: Added controller spawner nodes with proper delays:
```python
joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_broad', '--controller-manager', '/controller_manager'],
    parameters=[controller_config]
)

diff_drive_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['diff_cont', '--controller-manager', '/controller_manager'],
    parameters=[controller_config]
)
```

### 5. Timing Issues
**Problem**: Controllers were trying to spawn before controller_manager was ready.

**Fix**: Added event handlers and timers to delay controller spawning:
```python
delay_joint_state_broadcaster = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=spawn_entity,
        on_exit=[
            TimerAction(
                period=2.0,
                actions=[joint_state_broadcaster_spawner]
            )
        ]
    )
)
```

### 6. Additional Fixes in ros2_control.xacro
- Fixed plugin name from `ros2_control/GazeboSystem` to `gazebo_ros2_control/GazeboSystem`
- Fixed typo: `<paramaters>` → `<parameters>`
- Added `robot_param` and `robot_param_node` tags for proper robot description loading
- Added velocity state interfaces for better control

## Files Modified

1. `src/sim/launch/gazebo.launch.py`
   - Added xacro processing
   - Added controller spawners with delays
   - Added controller config loading

2. `src/sim/urdf/ros2_control.xacro`
   - Fixed all joint names
   - Fixed plugin name
   - Fixed typo
   - Added velocity state interfaces
   - Added robot_param configuration

3. `src/sim/config/controllers.yaml`
   - Updated wheel joint names

## How to Build and Run

### Build the package:
```bash
cd ~/Documents/pers-robotics/car-scratch
colcon build --packages-select sim
source install/setup.bash
```

### Launch Gazebo with the robot:
```bash
ros2 launch sim gazebo.launch.py
```

### Verify controllers are working:
```bash
# List available hardware interfaces
ros2 control list_hardware_interfaces

# List loaded controllers
ros2 control list_controllers

# List all controller_manager services
ros2 service list | grep controller_manager
```

### Control the robot:
The differential drive controller listens on:
```bash
/diff_cont/cmd_vel_unstamped
```

**Option 1: Keyboard Teleop (Recommended)**
```bash
# Install if not already installed
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleop with topic remapping
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Controls: `i`=forward, `,`=backward, `j`=left, `l`=right, `k`=stop

**Option 2: Manual velocity commands**
```bash
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

Run the SLAM
clarissa@clar-ubuntu:~$ ros2 launch slam_toolbox online_async_launch.py params_file:=./src/sim/config/mappers_params_online_async.yaml use_sim_time:=true


## Robot Configuration

### Wheel Parameters
- **Wheel radius**: 0.2m
- **Wheel separation**: 1.1m
- **Wheels**: 4-wheel differential drive (2 left, 2 right)

### Controllers
1. **joint_broad** (Joint State Broadcaster)
   - Publishes joint states to `/joint_states`

2. **diff_cont** (Differential Drive Controller)
   - Accepts velocity commands
   - Controls all 4 wheels (front and back on each side)
   - Base frame: `base_link`
   - Update rate: 30 Hz
   - Publish rate: 50 Hz

## Debugging Tips

If you encounter similar issues:

1. **Check if xacro is being processed**: Look for `import xacro` in launch files
2. **Verify joint names match**: Compare URDF joint names with ros2_control and controller config
3. **Check controller_manager is running**: `ros2 node list | grep controller_manager`
4. **View Gazebo plugins loaded**: Check terminal output when launching Gazebo
5. **Verify robot description**: `ros2 param get /robot_state_publisher robot_description`
6. **Check controller loading errors**: Look for `[ERROR] [controller_manager]: Could not initialize the controller` in launch terminal
7. **Use floats not integers**: ROS2 controllers expect `30.0` not `30` for numeric parameters
8. **List active controllers**: `ros2 control list_controllers` to verify all controllers loaded

## Key Takeaways

1. Always process `.xacro` files with `xacro.process_file()` before using them
2. Joint names must be consistent across URDF, ros2_control, and controller configs
3. Controllers need time to initialize - use delays or event handlers
4. The gazebo_ros2_control plugin automatically starts controller_manager when properly configured
5. Velocity state interfaces are recommended for velocity-based controllers like diff_drive

## References

- [ros2_control documentation](https://control.ros.org/)
- [gazebo_ros2_control documentation](https://github.com/ros-simulation/gazebo_ros2_control)
- [diff_drive_controller documentation](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
