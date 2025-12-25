# Chapter 6 Code Examples

**Chapter**: Unity Digital Twins
**Purpose**: Photorealistic robot visualization and digital twin implementation

---

## Important Note

**Requirements**:
- Unity Hub + Unity Editor 2021.3 LTS (or 2022.3 LTS)
- ROS 2 Humble
- Unity Robotics Hub packages
- Windows/macOS/Linux supported

---

## Quick Start

### 1. Install Unity

1. Download Unity Hub: [https://unity.com/download](https://unity.com/download)
2. Install Unity 2021.3 LTS via Unity Hub
3. Create new 3D (HDRP) project

### 2. Install Unity Robotics Hub

**In Unity Editor**:
```
Window → Package Manager → + → Add package from git URL

Add these URLs:
1. https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.ros-tcp-connector
2. https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

### 3. Install ROS-TCP Endpoint

```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

---

## Example Files

### `RobotStateVisualizer.cs`
**Demonstrates**: Real-time robot visualization from ROS 2 topics

**What it does**:
- Subscribes to `/joint_states` topic via ROS-TCP Connector
- Updates Unity ArticulationBody joints to match ROS positions
- Converts radians (ROS) to degrees (Unity)
- Maps joint names to Unity components

**Usage**:
1. Copy to `Assets/Scripts/` in Unity project
2. Attach to robot root GameObject
3. Assign ArticulationBody joints in Inspector
4. Launch ROS-TCP Endpoint: `ros2 run ros_tcp_endpoint default_server_endpoint`
5. Press Play in Unity
6. Publish joint states from ROS 2

**Expected**: Robot in Unity mirrors ROS joint positions in real-time

---

## Unity Project Structure

```
UnityProject/
├── Assets/
│   ├── Scripts/
│   │   └── RobotStateVisualizer.cs
│   ├── URDF/
│   │   └── simple_humanoid/ (imported from .urdf)
│   ├── Materials/
│   │   └── RobotMaterial.mat (PBR material)
│   └── Scenes/
│       └── RobotVisualization.unity
├── Packages/
│   ├── com.unity.robotics.ros-tcp-connector
│   └── com.unity.robotics.urdf-importer
└── ProjectSettings/
```

---

## Validation and Testing

```bash
# ROS 2 side: Launch endpoint
ros2 run ros_tcp_endpoint default_server_endpoint
# Expected: "Starting server on 0.0.0.0:10000"

# Unity side: Configure ROS Settings
# Robotics → ROS Settings
# ROS IP: 127.0.0.1, Port: 10000, Protocol: ROS 2

# Test: Publish joint states
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
  "header: {frame_id: ''} \
   name: ['neck_pan'] \
   position: [0.5]" \
  -r 10

# Expected: Robot head rotates in Unity
```

---

## Common Errors

**1. ROS-TCP Connector not found in Package Manager**
- Fix: Ensure git URL correct, check internet connection
- Alternative: Download from GitHub, add as local package

**2. URDF import fails: "No robot found"**
- Cause: URDF file syntax error
- Fix: Validate with `check_urdf` before importing

**3. Robot doesn't move in Unity**
- Diagnosis: Check Unity Console for connection errors
- Fixes:
  - Verify ROS-TCP Endpoint running (`ros2 node list` shows `/ros_tcp_endpoint`)
  - Check ROS Settings IP/port correct
  - Ensure joint names in script match URDF

**4. Unity freezes during import**
- Cause: Complex mesh with high poly count
- Fix: Simplify mesh in Blender, or use Unity primitives for collision

**5. Black materials after import**
- Cause: URDF materials not converted correctly
- Fix: Manually create PBR materials, assign to GameObjects

---

## System Requirements

**Minimum**:
- Unity: 2020.3+ (2021.3 LTS recommended)
- OS: Windows 10, macOS 10.13+, Ubuntu 20.04+
- CPU: 4-core
- RAM: 8GB
- GPU: DirectX 11/12 compatible (Intel HD 4000+)
- Disk: 20GB

**Recommended**:
- Unity: 2021.3 LTS
- CPU: 8-core
- RAM: 16GB
- GPU: NVIDIA GTX 1060 / AMD RX 580 (for HDRP ray tracing)
- Disk: 30GB SSD

---

## Related Chapter Content

- Section 1.3: Unity Robotics Hub Architecture
- Section 2.1: Installing Unity for Robotics
- Section 2.2: Importing URDF into Unity
- Section 2.3: ROS-Unity Communication Setup
- Section 2.4: Real-Time Robot Visualization
- Section 2.5: Creating Photorealistic Environments

---

## Additional Resources

- Unity Robotics Hub GitHub: [https://github.com/Unity-Technologies/Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- Unity Tutorials: [https://learn.unity.com/](https://learn.unity.com/)
- Free 3D Assets: [https://assetstore.unity.com/](https://assetstore.unity.com/)

---

**Last Updated**: 2025-12-23
**Tested On**: Unity 2021.3.16f1 LTS, Ubuntu 22.04, ROS 2 Humble
