# Chapter 4: URDF and Robot Modeling

**Part**: 2 - ROS 2 and Robot Software Architecture
**Estimated Reading Time**: 45-55 minutes
**Estimated Practice Time**: 4-5 hours (including URDF creation and visualization)

---

## Learning Objectives

By the end of this chapter, you will be able to:

**Conceptual Understanding**:
- Explain the purpose and structure of URDF (Unified Robot Description Format)
- Describe the key elements: links, joints, sensors, actuators
- Understand coordinate frames and transformations in robot modeling
- Differentiate between joint types (revolute, prismatic, fixed, continuous, planar, floating)
- Explain humanoid robot anatomy and kinematic chains
- Compare URDF and SDF (Simulation Description Format)

**Practical Skills**:
- Write a complete URDF file for a simple robot
- Define links with visual, collision, and inertial properties
- Create joints with appropriate types, limits, and dynamics
- Add sensors (cameras, LiDAR) to URDF models
- Visualize URDF models in RViz using joint_state_publisher
- Load and test URDF models in Gazebo simulator
- Debug common URDF errors using check_urdf tool

---

## Prerequisites

**Conceptual Prerequisites**:
- Chapter 3: ROS 2 Fundamentals (nodes, topics, TF transforms)
- Basic understanding of 3D coordinate systems (x, y, z, roll, pitch, yaw)
- Familiarity with rigid body transformations

**Technical Setup Prerequisites**:
- **ROS 2 Humble** installed (see Chapter 3, Section 2.1)
- **RViz** (included in ros-humble-desktop)
- **Gazebo** (optional, for simulation exercises)
- **urdfdom-py** (Python URDF parser)
- Text editor or IDE (VS Code, Sublime, vim)

---

## Part 1: Conceptual Foundations (Theory)

### 1.1 What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based specification for describing robot kinematics, dynamics, visual appearance, and collision geometry. It is the standard format for representing robots in ROS/ROS 2.

#### 1.1.1 Why URDF?

**Purpose**:
1. **Kinematics**: Define robot structure (links and joints) for motion planning and control
2. **Visualization**: Specify 3D meshes and colors for rendering in RViz/Gazebo
3. **Collision Detection**: Define simplified collision geometry for planning and simulation
4. **Dynamics**: Specify inertial properties (mass, inertia tensor) for physics simulation
5. **Sensors**: Attach sensors (cameras, LiDAR, IMU) to robot links with precise placement

**History**: URDF originated in ROS 1 (2009) and remains the primary robot description format in ROS 2. It's based on XML to be human-readable and easily parsable.

**Use Cases**:
- **Motion Planning**: MoveIt uses URDF for inverse kinematics and collision checking
- **Control**: ros2_control uses URDF to configure hardware interfaces
- **Simulation**: Gazebo loads URDF models for physics-based simulation
- **Visualization**: RViz displays URDF models with joint states

#### 1.1.2 URDF vs. SDF vs. MJCF

| Format | Full Name | Origin | Strengths | Weaknesses |
|--------|-----------|--------|-----------|------------|
| **URDF** | Unified Robot Description Format | ROS (2009) | ROS integration, widespread adoption, simple syntax | Tree structure only (no loops), limited sensor plugins |
| **SDF** | Simulation Description Format | Gazebo (2012) | Supports loops, world descriptions, advanced plugins | More complex, less human-readable |
| **MJCF** | MuJoCo XML Format | MuJoCo (2012) | Efficient contact dynamics, reinforcement learning | MuJoCo-specific, less ROS support |

**Key Difference**: URDF models **robots** (kinematic trees), while SDF models **worlds** (robots + environment + physics).

**Conversion**: ROS 2 can automatically convert URDF → SDF when loading into Gazebo.

---

### 1.2 URDF Structure and Syntax

#### 1.2.1 Core Elements

A URDF file consists of three primary elements:

1. **`<robot>`**: Root element (one per file)
2. **`<link>`**: Rigid body (physical part of robot)
3. **`<joint>`**: Connection between two links (defines motion)

**Minimal URDF Example**:
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Moving link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

**Kinematic Tree**: URDF represents robots as **trees** (no kinematic loops):
```
        base_link (root)
            |
        arm_link (child)
            |
      end_effector_link
```

**Why Tree Structure?**: Simplifies forward kinematics (FK) computation. For robots with loops (e.g., parallel manipulators), use **virtual joints** or switch to SDF.

---

#### 1.2.2 Links: Physical Bodies

A **link** represents a rigid body in the robot. Each link has three optional properties:

**1. Visual** (appearance in RViz):
```xml
<link name="body">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Pose relative to link frame -->
    <geometry>
      <box size="0.6 0.4 0.2"/>        <!-- Geometry: box, cylinder, sphere, mesh -->
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>          <!-- RGBA (0-1 range) -->
    </material>
  </visual>
</link>
```

**Geometry Types**:
- **`<box size="x y z"/>`**: Rectangular box (meters)
- **`<cylinder radius="r" length="l"/>`**: Cylinder along z-axis
- **`<sphere radius="r"/>`**: Sphere
- **`<mesh filename="package://my_robot/meshes/arm.stl" scale="1 1 1"/>`**: Custom 3D mesh (STL, DAE, OBJ)

**2. Collision** (for collision checking):
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.6 0.4 0.2"/>  <!-- Often simplified vs. visual -->
  </geometry>
</collision>
```

**Best Practice**: Use simplified geometry for collision (boxes, cylinders) even if visual uses complex meshes → faster collision checking.

**3. Inertial** (for physics simulation):
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Center of mass -->
  <mass value="10.0"/>                <!-- kg -->
  <inertia ixx="0.1" ixy="0.0" ixz="0.0"
           iyy="0.1" iyz="0.0" izz="0.1"/>  <!-- kg·m² -->
</inertial>
```

**Inertia Tensor** (3×3 symmetric matrix):
$$
\mathbf{I} = \begin{bmatrix}
I_{xx} & I_{xy} & I_{xz} \\
I_{xy} & I_{yy} & I_{yz} \\
I_{xz} & I_{yz} & I_{zz}
\end{bmatrix}
$$

**For simple shapes**:
- **Box** $(a, b, c)$: $I_{xx} = \frac{m}{12}(b^2 + c^2)$, $I_{yy} = \frac{m}{12}(a^2 + c^2)$, $I_{zz} = \frac{m}{12}(a^2 + b^2)$
- **Cylinder** $(r, h)$ along z: $I_{xx} = I_{yy} = \frac{m}{12}(3r^2 + h^2)$, $I_{zz} = \frac{mr^2}{2}$
- **Sphere** $(r)$: $I_{xx} = I_{yy} = I_{zz} = \frac{2mr^2}{5}$

**Off-diagonal terms** ($I_{xy}, I_{xz}, I_{yz}$) are non-zero for asymmetric objects or when center of mass is not at link origin.

---

#### 1.2.3 Joints: Connections and Motion

A **joint** connects two links and defines their relative motion.

**Joint Definition**:
```xml
<joint name="shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0.15 0.3" rpy="0 0 0"/>  <!-- Child frame relative to parent -->
  <axis xyz="0 1 0"/>                      <!-- Rotation axis (unit vector) -->
  <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  <dynamics damping="0.7" friction="0.1"/>
</joint>
```

**Joint Types**:

| Type | DOF | Description | Use Case |
|------|-----|-------------|----------|
| **revolute** | 1 | Rotation around axis (limited range) | Robot joints (shoulder, elbow, knee) |
| **continuous** | 1 | Rotation around axis (unlimited range) | Wheels |
| **prismatic** | 1 | Translation along axis (limited range) | Linear actuators, grippers |
| **fixed** | 0 | No motion (rigid connection) | Sensor mounts, fixed body parts |
| **floating** | 6 | Unconstrained 6-DOF (3 translation + 3 rotation) | Mobile bases (rarely used in URDF) |
| **planar** | 2 | 2D translation in plane + 1 rotation around normal | Rarely used |

**Revolute Joint Parameters**:
- **`<axis xyz="x y z"/>`**: Rotation axis in **child** frame (unit vector)
  - Example: `<axis xyz="0 1 0"/>` rotates around Y-axis
- **`<limit>`**: Joint constraints
  - `lower`, `upper`: Angle limits (radians, e.g., -π to π)
  - `effort`: Max torque (N·m)
  - `velocity`: Max angular velocity (rad/s)
- **`<dynamics>`**: Friction and damping
  - `damping`: Velocity-dependent damping (N·m·s/rad)
  - `friction`: Coulomb friction (N·m)

**Prismatic Joint Parameters**:
```xml
<joint name="gripper_finger" type="prismatic">
  <axis xyz="1 0 0"/>  <!-- Translation along X -->
  <limit lower="0" upper="0.05" effort="10" velocity="0.1"/>  <!-- meters -->
</joint>
```

**Fixed Joint** (no motion):
```xml
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.1" rpy="0 0 0"/>  <!-- Fixed offset -->
</joint>
```

---

#### 1.2.4 Coordinate Frames and Transforms

**Frame Convention** (ROS REP 103):
- **X**: Forward (red)
- **Y**: Left (green)
- **Z**: Up (blue)
- **Right-hand rule**: X × Y = Z

**Link Frame**: Each link has a coordinate frame at its origin (defined by `<origin>` in link's parent joint).

**Joint Frame**: Defined by `<origin>` in joint definition:
- **Position**: `xyz="x y z"` (meters)
- **Orientation**: `rpy="roll pitch yaw"` (radians, intrinsic ZYX Euler angles)

**Roll-Pitch-Yaw (RPY)** to Rotation Matrix:
$$
\mathbf{R} = \mathbf{R}_z(\text{yaw}) \cdot \mathbf{R}_y(\text{pitch}) \cdot \mathbf{R}_x(\text{roll})
$$

**Example**: Camera mounted 10cm forward, 5cm up, pitched down 30°:
```xml
<origin xyz="0.1 0 0.05" rpy="0 -0.524 0"/>  <!-- pitch = -30° = -0.524 rad -->
```

**Transform Tree** (computed by TF in ROS):
```
world → base_link → torso → head → camera_link
```

ROS 2 `tf2` library maintains transform tree, allowing queries like "What is camera pose in world frame?"

---

### 1.3 Humanoid Robot Anatomy

#### 1.3.1 Kinematic Structure

A typical humanoid has ~30-50 DOF (Degrees of Freedom):

**Body Segments**:
1. **Base/Pelvis** (root link, 6 DOF if floating)
2. **Torso** (0-3 DOF: yaw, pitch, roll or fixed)
3. **Head** (2-3 DOF: pan, tilt, roll)
4. **Arms** (2× 7 DOF each = 14 DOF total)
   - Shoulder: 3 DOF (pitch, roll, yaw)
   - Elbow: 1 DOF (pitch)
   - Wrist: 3 DOF (pitch, roll, yaw)
   - (Hand: 1-20 DOF for fingers, often simplified)
5. **Legs** (2× 6 DOF each = 12 DOF total)
   - Hip: 3 DOF (pitch, roll, yaw)
   - Knee: 1 DOF (pitch)
   - Ankle: 2 DOF (pitch, roll)

**Total**: 6 (base) + 3 (torso) + 3 (head) + 14 (arms) + 12 (legs) = **38 DOF** (typical research humanoid)

**Commercial Examples**:
- **Atlas (Boston Dynamics)**: 28 DOF (no floating base in model, no hand fingers)
- **Valkyrie (NASA)**: 44 DOF (includes 6 per hand)
- **Tesla Optimus**: 40 DOF (28 actuated + 12 passive in hands)

#### 1.3.2 Kinematic Chains

**Serial Chain**: Sequence of links connected by joints (e.g., arm: shoulder → elbow → wrist → end-effector)

**Parallel Chain**: Multiple paths from base to end-effector (rare in humanoids, common in robotic hands)

**Open Chain**: Has free end (e.g., arm with gripper)

**Closed Chain**: Forms loop (e.g., parallel linkage) — **not directly supported in URDF**, use virtual joints

**Humanoid Kinematic Chains**:
```
                    Head
                      |
        Left Arm - Torso - Right Arm
                      |
        Left Leg - Pelvis - Right Leg
```

Each limb is a **serial chain** branching from torso/pelvis (tree structure).

---

### 1.4 Adding Sensors to URDF

Sensors are defined using **Gazebo plugins** (not part of base URDF spec).

#### 1.4.1 Camera Sensor

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.03 0.08 0.03"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for camera (simulation only) -->
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>  <!-- FPS -->
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60° in radians -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>  <!-- Min distance (m) -->
        <far>300</far>     <!-- Max distance (m) -->
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**Key Parameters**:
- `horizontal_fov`: Field of view in radians (60° = 1.047 rad)
- `update_rate`: Frame rate (Hz)
- `clip`: Near/far rendering planes

#### 1.4.2 LiDAR Sensor

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>  <!-- Show rays in Gazebo GUI -->
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>        <!-- Points per scan -->
          <resolution>1</resolution>
          <min_angle>-1.57</min_angle>  <!-- -90° -->
          <max_angle>1.57</max_angle>   <!-- +90° -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>  <!-- 1cm accuracy -->
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

#### 1.4.3 IMU Sensor

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- Hz -->
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>  <!-- rad/s -->
          </noise>
        </x>
        <!-- ... y, z similar -->
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.1</stddev>  <!-- m/s² -->
          </noise>
        </x>
        <!-- ... y, z similar -->
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

---

## Part 2: Hands-On Implementation (Practice)

### 2.1 Setting Up URDF Development Environment

#### 2.1.1 Install Required Packages

```bash
# ROS 2 Humble (see Chapter 3 if not installed)
source /opt/ros/humble/setup.bash

# Install URDF/RViz tools
sudo apt install ros-humble-urdf-tutorial ros-humble-joint-state-publisher-gui

# Install URDF Python parser (for validation)
sudo apt install python3-urdfdom-py

# Install Gazebo (optional, for simulation)
sudo apt install ros-humble-gazebo-ros-pkgs

# Verify installation
check_urdf --help  # Should show usage
```

#### 2.1.2 Create URDF Package

```bash
# Create workspace (if not exists)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_cmake my_robot_description

# Create URDF directory
cd my_robot_description
mkdir urdf launch meshes

# Package structure:
# my_robot_description/
# ├── CMakeLists.txt
# ├── package.xml
# ├── urdf/           # URDF files
# ├── launch/         # Launch files
# └── meshes/         # 3D meshes (STL, DAE)
```

#### 2.1.3 Update CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

# Install URDF files
install(DIRECTORY urdf launch meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

### 2.2 Practice Example 1: Simple Mobile Robot

#### Overview
Create a differential drive robot with cylindrical body and two wheels.

Create file: `~/ros2_ws/src/my_robot_description/urdf/simple_robot.urdf`

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base Link (robot body) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.027" ixy="0.0" ixz="0.0"
               iyy="0.027" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Left Wheel Link -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>  <!-- Rotate 90° to align with Y -->
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>
      <inertia ixx="0.0007" ixy="0.0" ixz="0.0"
               iyy="0.0007" iyz="0.0" izz="0.00125"/>
    </inertial>
  </link>

  <!-- Left Wheel Joint (continuous rotation) -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="0 0 0"/>  <!-- Left side -->
    <axis xyz="0 1 0"/>  <!-- Rotate around Y -->
  </joint>

  <!-- Right Wheel Link -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0" rpy="1.5707 0 0"/>
      <inertia ixx="0.0007" ixy="0.0" ixz="0.0"
               iyy="0.0007" iyz="0.0" izz="0.00125"/>
    </inertial>
  </link>

  <!-- Right Wheel Joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="0 0 0"/>  <!-- Right side -->
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Caster Wheel (passive, for stability) -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.025"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0"
               iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <!-- Caster Joint (fixed, or use continuous for free rotation) -->
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.15 0 -0.075" rpy="0 0 0"/>  <!-- Back center -->
  </joint>

</robot>
```

#### Validate URDF

```bash
cd ~/ros2_ws/src/my_robot_description/urdf

# Check syntax
check_urdf simple_robot.urdf

# Expected output:
# robot name is: simple_robot
# ---------- Successfully Parsed XML ---------------
# root Link: base_link has 3 child(ren)
#     child(1):  left_wheel
#     child(2):  right_wheel
#     child(3):  caster_wheel
```

#### Visualize in RViz

Create launch file: `~/ros2_ws/src/my_robot_description/launch/display.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    urdf_file = PathJoinSubstitution(
        [FindPackageShare('my_robot_description'), 'urdf', 'simple_robot.urdf']
    )

    return LaunchDescription([
        # Robot State Publisher (publishes TF from URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
        ),

        # Joint State Publisher GUI (interactive sliders)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('my_robot_description'), 'rviz', 'robot.rviz'
            ])],
        ),
    ])
```

**Build and Launch**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash

ros2 launch my_robot_description display.launch.py
```

**Expected Result**: RViz opens showing the robot. Use joint_state_publisher_gui sliders to rotate wheels.

---

### 2.3 Practice Example 2: Simple Humanoid Robot

#### Overview
Create a simplified humanoid upper body with torso, head, and arms.

Create file: `~/ros2_ws/src/my_robot_description/urdf/simple_humanoid.urdf`

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Torso (base link) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.4 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.4 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="20.0"/>
      <inertia ixx="1.33" ixy="0.0" ixz="0.0"
               iyy="1.0" iyz="0.0" izz="0.67"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0"
               iyy="0.008" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <!-- Head Joint (revolute, pan left-right) -->
  <joint name="neck_pan" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>  <!-- Top of torso -->
    <axis xyz="0 0 1"/>  <!-- Rotate around Z (yaw) -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2.0"/>
  </joint>

  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>  <!-- Center geometry -->
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.0125" ixy="0.0" ixz="0.0"
               iyy="0.0125" iyz="0.0" izz="0.0012"/>
    </inertial>
  </link>

  <!-- Left Shoulder Joint (revolute, pitch forward-back) -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0.2 0.2" rpy="0 0 0"/>  <!-- Left side of torso -->
    <axis xyz="0 1 0"/>  <!-- Rotate around Y (pitch) -->
    <limit lower="-3.14" upper="3.14" effort="30" velocity="2.0"/>
  </joint>

  <!-- Left Forearm -->
  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.0055" ixy="0.0" ixz="0.0"
               iyy="0.0055" iyz="0.0" izz="0.00045"/>
    </inertial>
  </link>

  <!-- Left Elbow Joint (revolute, bend arm) -->
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>  <!-- Bottom of upper arm -->
    <axis xyz="0 1 0"/>  <!-- Rotate around Y (pitch) -->
    <limit lower="0" upper="2.356" effort="20" velocity="2.0"/>  <!-- 0 to 135° -->
  </joint>

  <!-- Right Arm (mirror of left) -->
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.0125" ixy="0.0" ixz="0.0"
               iyy="0.0125" iyz="0.0" izz="0.0012"/>
    </inertial>
  </link>

  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 -0.2 0.2" rpy="0 0 0"/>  <!-- Right side -->
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="30" velocity="2.0"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.125" rpy="0 0 0"/>
      <inertia ixx="0.0055" ixy="0.0" ixz="0.0"
               iyy="0.0055" iyz="0.0" izz="0.00045"/>
    </inertial>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.356" effort="20" velocity="2.0"/>
  </joint>

</robot>
```

#### Visualize Humanoid

```bash
# Validate
check_urdf simple_humanoid.urdf

# Launch (update launch file to use simple_humanoid.urdf)
ros2 launch my_robot_description display.launch.py
```

**Use Joint State Publisher GUI** to move arms and head interactively.

---

### 2.4 Adding a Camera Sensor

Extend `simple_humanoid.urdf` with camera on head:

```xml
<!-- Add after head link definition -->

<!-- Camera Link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.06 0.02"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.06 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.000017" ixy="0.0" ixz="0.0"
             iyy="0.000002" iyz="0.0" izz="0.000017"/>
  </inertial>
</link>

<!-- Camera Joint (fixed to head) -->
<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.08 0 0.05" rpy="0 0 0"/>  <!-- Front of head, slightly up -->
</joint>

<!-- Gazebo camera plugin -->
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60° -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/simple_humanoid</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Test in Gazebo**:
```bash
# Launch Gazebo with robot
gazebo --verbose -s libgazebo_ros_factory.so

# Spawn robot
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity simple_humanoid

# View camera feed
ros2 run rqt_image_view rqt_image_view /simple_humanoid/camera/image_raw
```

---

### 2.5 Common URDF Errors and Debugging

#### Error 1: Invalid XML Syntax

**Error**:
```
Error parsing XML: mismatched tag at line 45
```

**Cause**: Unclosed tag (e.g., `<link>` without `</link>`)

**Fix**: Use XML validator or IDE with XML support (VS Code with XML extension)

#### Error 2: Disconnected Links

**Error**:
```
Error: link 'arm_link' is not connected to the root
```

**Cause**: No joint connects `arm_link` to rest of robot tree

**Fix**: Ensure all links (except root) have exactly one parent joint

#### Error 3: Invalid Inertia Tensor

**Error**:
```
Warning: Inertia matrix is not positive definite
```

**Cause**: Incorrect inertia values (e.g., negative, or $I_{xx} < I_{yy} + I_{zz}$)

**Fix**: Recalculate using correct formulas, or use MeshLab/Blender to compute from mesh

#### Error 4: Joint Axis Not Unit Vector

**Error**:
```
Warning: Joint axis is not normalized
```

**Cause**: `<axis xyz="1 0 2"/>` has length ≠ 1

**Fix**: Normalize vector: $\hat{v} = \frac{\vec{v}}{|\vec{v}|}$ → `<axis xyz="0.447 0 0.894"/>`

#### Error 5: Gazebo Plugin Not Loading

**Error**:
```
[Err] [Plugin.hh:187] Failed to load plugin libgazebo_ros_camera.so
```

**Cause**: Gazebo ROS plugins not installed

**Fix**: `sudo apt install ros-humble-gazebo-ros-pkgs`

---

## Part 3: Optional Hardware Deployment

### 3.1 Loading URDF to Real Robot

For robots with ros2_control hardware interface:

1. **Add ros2_control tags** to URDF:
```xml
<ros2_control name="my_robot" type="system">
  <hardware>
    <plugin>my_robot_hardware/MyRobotHardware</plugin>
  </hardware>
  <joint name="left_shoulder_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- Repeat for all actuated joints -->
</ros2_control>
```

2. **Launch controller manager**:
```bash
ros2 launch my_robot_description robot.launch.py
```

3. **Load controllers**:
```bash
ros2 control load_controller joint_trajectory_controller
ros2 control set_controller_state joint_trajectory_controller start
```

### 3.2 Sim-to-Real Transfer

**Challenges**:
- **Inertia mismatch**: Simulated inertia rarely matches real robot exactly
- **Friction**: Real joints have complex friction (Coulomb, viscous, stiction) not captured by simple damping
- **Flexibility**: Real robots have link/joint compliance; URDF assumes rigid bodies
- **Sensor noise**: Simulation has perfect sensors unless explicit noise added

**Best Practices**:
1. Measure real robot parameters (mass, dimensions, joint limits)
2. Add realistic sensor noise in Gazebo
3. Test controllers in simulation before deploying
4. Use system identification to refine inertial parameters

---

## Review Questions

**Question 1** (URDF Structure): What are the three primary elements of a URDF file? Explain the role of each.

**Question 2** (Joint Types): A robot gripper needs to open/close linearly by 5cm. Which joint type should you use? Write the XML for this joint including limits.

**Question 3** (Coordinate Frames): A camera is mounted on a robot head. The head link's origin is at the neck. The camera should be 8cm forward, 3cm up, and pitched down 20°. Write the `<origin>` tag for the camera joint.

**Question 4** (Inertia Calculation): Calculate the inertia tensor for a solid cylinder with mass 2kg, radius 0.05m, and length 0.3m, oriented along the Z-axis. Use formulas: $I_{xx} = I_{yy} = \frac{m}{12}(3r^2 + h^2)$, $I_{zz} = \frac{mr^2}{2}$.

**Question 5** (Debugging): Your URDF loads in RViz but crashes Gazebo with "Inertia matrix not positive definite." What could be wrong and how do you fix it?

---

## Hands-On Exercises

### Exercise 1: Four-Wheeled Robot

**Task**: Extend the simple mobile robot (Section 2.2) to have four wheels instead of two (plus caster).

**Requirements**:
- 4 wheels arranged in rectangular pattern (front-left, front-right, rear-left, rear-right)
- All wheels are continuous joints
- Add a LiDAR sensor on top center of base_link

**Solution Guidance**:
- Copy `simple_robot.urdf`, rename to `four_wheel_robot.urdf`
- Duplicate left/right wheel links → front_left, front_right, rear_left, rear_right
- Adjust joint origins: front at `xyz="0.1 ±0.15 -0.05"`, rear at `xyz="-0.1 ±0.15 -0.05"`
- Add LiDAR link at `xyz="0 0 0.1"` (top of base)
- Include Gazebo plugin for LiDAR (Section 1.4.2)

---

### Exercise 2: Humanoid with Legs

**Task**: Extend `simple_humanoid.urdf` to add legs (simplified: hip, thigh, shin, foot).

**Requirements**:
- Two legs (left and right), each with 3 DOF:
  - Hip pitch (thigh rotates forward/back)
  - Knee pitch (shin bends)
  - Ankle pitch (foot tilts)
- Use revolute joints with realistic limits:
  - Hip: -30° to 120°
  - Knee: 0° to 135°
  - Ankle: -45° to 45°

**Solution Guidance**:
- Create links: `left_thigh`, `left_shin`, `left_foot` (and right equivalents)
- Thigh dimensions: cylinder radius 0.05m, length 0.4m
- Shin dimensions: cylinder radius 0.04m, length 0.4m
- Foot dimensions: box 0.08 × 0.15 × 0.03m
- Hip joint origin: `xyz="0 ±0.15 -0.25"` (below torso)
- Knee at bottom of thigh, ankle at bottom of shin

---

### Exercise 3: URDF from CAD Model

**Task**: Convert a CAD model (STL file) to URDF.

**Steps**:
1. Export STL from CAD software (SolidWorks, Fusion 360, Blender)
2. Create URDF with `<mesh filename="package://my_robot_description/meshes/part.stl"/>`
3. Compute inertial properties:
   - Use Blender: Select mesh → Properties → Physics → Rigid Body → Calculate Mass
   - Or MeshLab: Filters → Quality Measure → Compute Geometric Measures
4. Set visual and collision geometry (collision can be simplified box/cylinder)
5. Validate with `check_urdf` and visualize in RViz

**Example URDF snippet**:
```xml
<link name="custom_part">
  <visual>
    <geometry>
      <mesh filename="package://my_robot_description/meshes/custom_part.stl" scale="1 1 1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.1 0.2"/>  <!-- Simplified -->
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0"
             iyy="0.001" iyz="0.0" izz="0.0005"/>
  </inertial>
</link>
```

---

## Key Takeaways

1. **URDF is XML-based**: Human-readable robot description format standard in ROS/ROS 2 for kinematics, dynamics, visualization, collision.

2. **Three core elements**: `<robot>` (root), `<link>` (rigid body with visual/collision/inertial), `<joint>` (connection defining motion between links).

3. **Six joint types**: Revolute (limited rotation), continuous (unlimited rotation), prismatic (linear), fixed (rigid), floating (6-DOF), planar (2D motion).

4. **Kinematic tree structure**: URDF represents robots as trees (no loops) with root link and parent-child relationships via joints.

5. **Coordinate frames matter**: Each link/joint has origin with position (xyz) and orientation (rpy roll-pitch-yaw); ROS uses X forward, Y left, Z up.

6. **Inertia is critical**: Correct mass and inertia tensor required for realistic physics simulation; use formulas or CAD tools to compute.

7. **Sensors via Gazebo plugins**: Camera, LiDAR, IMU added using `<gazebo>` tags with ROS 2 plugins (not part of base URDF spec).

8. **Validation tools**: `check_urdf` checks syntax/connectivity, RViz visualizes with joint_state_publisher, Gazebo tests physics/dynamics.

9. **Humanoid anatomy**: Typical 30-50 DOF (base 6, torso 3, head 3, arms 14, legs 12) organized as serial chains branching from torso/pelvis.

10. **Sim-to-real gap**: Simulated URDF parameters (inertia, friction, sensor noise) must match real robot; use system identification and iterative refinement.

---

## References

(References section to be created in separate file Ch4.md references)

---

## Answer Key

**Answer 1**: Three primary elements:
- **`<robot>`**: Root element defining robot name (one per file)
- **`<link>`**: Rigid body representing physical parts with visual geometry (appearance), collision geometry (for planning), and inertial properties (mass, inertia for simulation)
- **`<joint>`**: Connection between two links defining motion constraints (parent/child relationship, joint type, axis, limits)

**Answer 2**: Use **prismatic** joint (linear translation). XML:
```xml
<joint name="gripper_finger_joint" type="prismatic">
  <parent link="gripper_base"/>
  <child link="gripper_finger"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- Translate along X -->
  <limit lower="0" upper="0.05" effort="10" velocity="0.1"/>  <!-- 0-5cm -->
</joint>
```

**Answer 3**: Forward 8cm = 0.08m X, up 3cm = 0.03m Z, pitch down 20° = -0.349 radians:
```xml
<origin xyz="0.08 0 0.03" rpy="0 -0.349 0"/>
```

**Answer 4**: Cylinder m=2kg, r=0.05m, h=0.3m along Z:
- $I_{xx} = I_{yy} = \frac{2}{12}(3 \times 0.05^2 + 0.3^2) = 0.01625 \, \text{kg·m}^2$
- $I_{zz} = \frac{2 \times 0.05^2}{2} = 0.0025 \, \text{kg·m}^2$
- Off-diagonal: $I_{xy} = I_{xz} = I_{yz} = 0$

XML:
```xml
<inertia ixx="0.01625" ixy="0.0" ixz="0.0"
         iyy="0.01625" iyz="0.0" izz="0.0025"/>
```

**Answer 5**: **Cause**: Invalid inertia tensor (e.g., negative values, wrong formula, or $I_{xx} < |I_{yy} - I_{zz}|$ violating triangle inequality). **Fix**: Recalculate using correct formulas for link geometry, ensure all diagonal elements positive, verify triangle inequality ($I_{xx} \leq I_{yy} + I_{zz}$, etc.), use CAD tool (Blender, MeshLab) to compute from mesh if geometry is complex.

---

**End of Chapter 4**

**Next Chapter Preview**: Chapter 5 will cover Gazebo simulation environment, spawning robots, physics configuration, and sensor simulation for testing before hardware deployment.

---

**Last Updated**: 2025-12-23
**Tested On**: Ubuntu 22.04, ROS 2 Humble, Gazebo 11
