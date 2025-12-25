# Chapter 5 Code Examples

**Chapter**: Gazebo Simulation
**Purpose**: Hands-on physics-based simulation, world creation, sensor testing

---

## Important Note

**Requirements**: ROS 2 Humble + Gazebo Classic 11 (see Chapter 5, Section 2.1)

---

## Quick Start

### 1. Install Gazebo

```bash
source /opt/ros/humble/setup.bash
sudo apt install gazebo ros-humble-gazebo-ros-pkgs
gazebo --version  # Verify: should show 11.x
```

### 2. Create Simulation Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_simulation
cd my_robot_simulation
mkdir worlds launch scripts
```

### 3. Copy Example Files

Copy SDF world files and Python scripts from Chapter 5 to appropriate directories.

---

## Example Files

### `simple_world.world`
**Demonstrates**: SDF world creation with physics, lighting, static obstacles

**What it includes**:
- Physics configuration (ODE, 1ms timestep, gravity -9.81)
- Directional lighting with shadows
- Ground plane
- Static obstacles (red box, green cylinder)
- Camera pose for isometric view

**Launch**:
```bash
gazebo worlds/simple_world.world
```

**Expected**: Gazebo GUI showing world with ground, red box at (2,0), green cylinder at (-2,2)

---

### `teleop_humanoid.py`
**Demonstrates**: Keyboard teleoperation for joint control

**What it does**:
- Publishes joint position commands to ros2_control controllers
- Keyboard input: q/a (shoulder), w/s (elbow), e/d (neck), space (reset)
- Non-blocking keyboard capture with termios

**Prerequisites**:
- Robot URDF with ros2_control tags (see Chapter 4, Section 3.1)
- Controllers loaded in Gazebo

**Running**:
```bash
# Terminal 1: Launch Gazebo with robot + controllers
ros2 launch my_robot_simulation robot.launch.py

# Terminal 2: Run teleop
ros2 run my_robot_simulation teleop_humanoid
```

**Note**: Full ros2_control setup beyond this chapter scope; use `joint_state_publisher_gui` as simpler alternative.

---

## Validation and Testing

```bash
# Validate world file
gz sdf -k worlds/simple_world.world

# Check Gazebo is running
gz stats

# List Gazebo topics (ROS 2)
ros2 topic list | grep gazebo

# Spawn robot from URDF
gazebo &
ros2 run gazebo_ros spawn_entity.py -file urdf/simple_robot.urdf -entity test_robot -x 0 -y 0 -z 0.5
```

---

## Common Errors

**1. gazebo: command not found**
- Fix: `sudo apt install gazebo`

**2. Slow simulation (RTF < 1.0)**
- Fix: Simplify collision geometry, increase timestep to 0.002s, reduce sensor rates

**3. Robot falls through ground**
- Fix: Increase contact stiffness `<kp>1e7</kp>`, increase solver iterations `<iters>100</iters>`

**4. Black screen / GPU errors**
- Fix: `export LIBGL_ALWAYS_SOFTWARE=1` before `gazebo`

**5. Sensor data not publishing**
- Fix: Check `ros2 topic list`, verify Gazebo plugin loaded (check console for errors)

---

## File Structure

```
my_robot_simulation/
├── CMakeLists.txt
├── package.xml
├── worlds/
│   └── simple_world.world
├── launch/
│   └── simulation.launch.py
└── scripts/
    └── teleop_humanoid.py
```

---

## Related Chapter Content

- Section 1.2: Physics Engines (ODE, Bullet, DART)
- Section 2.2: Creating Simple World
- Section 2.3: Spawning Robots
- Section 2.4: Reading Sensor Data
- Section 2.5: Basic Teleoperation
- Section 2.6: Debugging and Performance

---

**Last Updated**: 2025-12-23
**Tested On**: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11
