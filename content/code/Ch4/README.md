# Chapter 4 Code Examples

**Chapter**: URDF and Robot Modeling
**Purpose**: Hands-on URDF creation, visualization, and simulation

---

## Important Note

**ROS 2 Humble Required**: These examples require ROS 2 Humble with RViz and optionally Gazebo (see Chapter 3).

---

## Quick Start

### 1. Install Dependencies

```bash
# ROS 2 Humble (see Chapter 3)
source /opt/ros/humble/setup.bash

# Install URDF tools
sudo apt install ros-humble-urdf-tutorial ros-humble-joint-state-publisher-gui

# Install Gazebo (optional)
sudo apt install ros-humble-gazebo-ros-pkgs

# Verify
check_urdf --help
```

### 2. Create Package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_description
cd my_robot_description
mkdir urdf launch meshes
```

### 3. Copy URDF Files

Copy `simple_robot.urdf` and `simple_humanoid.urdf` from Chapter 4 to `urdf/` directory.

### 4. Build and Visualize

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash

# Visualize in RViz
ros2 launch my_robot_description display.launch.py
```

---

## Example URDFs

### `simple_robot.urdf`
- Differential drive robot with two wheels + caster
- Demonstrates: links, continuous joints, cylinder/sphere/box geometry
- ~100 lines

### `simple_humanoid.urdf`
- Upper body humanoid (torso, head, arms)
- Demonstrates: revolute joints with limits, kinematic chains
- ~150 lines

### Camera-equipped humanoid
- Extends simple_humanoid with camera sensor
- Demonstrates: fixed joints, Gazebo plugins
- See Chapter 4, Section 2.4

---

## Validation and Visualization

```bash
# Validate URDF syntax
check_urdf urdf/simple_robot.urdf

# Expected output:
# robot name is: simple_robot
# ---------- Successfully Parsed XML ---------------
# root Link: base_link has 3 child(ren)

# Visualize in RViz with joint sliders
ros2 launch my_robot_description display.launch.py

# Load in Gazebo
gazebo --verbose
ros2 run gazebo_ros spawn_entity.py -file urdf/simple_robot.urdf -entity my_robot
```

---

## Common Errors

**1. check_urdf: command not found**
- Fix: `sudo apt install liburdfdom-tools`

**2. Error: link not connected to root**
- Cause: Missing joint connecting link to tree
- Fix: Ensure all links (except root) have parent joint

**3. Inertia matrix not positive definite**
- Cause: Invalid inertia values
- Fix: Recalculate using formulas in Chapter 4, Section 1.2.2

**4. RViz doesn't show robot**
- Cause: robot_state_publisher not running or TF frames missing
- Fix: Check launch file includes robot_state_publisher node

**5. Gazebo plugin not loading**
- Fix: `sudo apt install ros-humble-gazebo-ros-pkgs`

---

## File Structure

```
my_robot_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── simple_robot.urdf
│   ├── simple_humanoid.urdf
│   └── humanoid_with_camera.urdf
├── launch/
│   └── display.launch.py
└── meshes/
    └── (STL/DAE files)
```

---

## Related Chapter Content

- Section 1.2: URDF Structure (links, joints, sensors)
- Section 2.2: Simple Mobile Robot Example
- Section 2.3: Simple Humanoid Example
- Section 2.4: Adding Camera Sensor
- Section 2.5: Common URDF Errors

---

**Last Updated**: 2025-12-23
**Tested On**: Ubuntu 22.04, ROS 2 Humble, RViz2
