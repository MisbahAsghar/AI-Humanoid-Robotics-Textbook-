# Chapter 7 Code Examples

**Chapter**: Isaac Platform Overview
**Purpose**: GPU-accelerated simulation, parallel training, synthetic data generation

---

## Important Note

**Requirements**:
- **NVIDIA GPU**: RTX 2070+ (RTX 3060+ recommended)
- **Ubuntu 22.04 LTS** (primary platform)
- **NVIDIA Driver**: 525.x or newer
- **RAM**: 32GB recommended (16GB minimum)
- **Disk**: 50GB free space
- **ROS 2 Humble** (optional, for ROS integration)

**Alternative**: Use cloud instances (AWS g4dn, GCP with T4/A100) if no local GPU.

---

## Quick Start

### 1. Install Isaac Sim

**Omniverse Launcher** (easiest):
```bash
# Download from https://www.nvidia.com/en-us/omniverse/download/
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# In Launcher: Exchange → Isaac Sim → Install
```

**Docker** (advanced):
```bash
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
docker run --rm -it --gpus all nvcr.io/nvidia/isaac-sim:2023.1.1
```

### 2. Verify Installation

```bash
# Check GPU
nvidia-smi  # Should show RTX GPU with driver 525+

# Launch Isaac Sim
# From Omniverse Launcher: Library → Isaac Sim → Launch
```

---

## Example Scripts

### `parallel_envs_example.py`
**Demonstrates**: Creating 100 parallel robot environments for RL training

**What it does**:
- Launches Isaac Sim headless (no GUI for speed)
- Creates 100 robot instances in 10×10 grid
- Each robot at 2m spacing: (0,0), (2,0), (4,0), ...
- Runs 1000 simulation timesteps
- Demonstrates: World creation, reference adding, prim manipulation

**Running**:
```bash
# Isaac Sim must be installed
python parallel_envs_example.py
```

**Expected**: Console shows "Creating 100 environments...", simulation runs at 100+ FPS

---

### Synthetic Data Generation (Replicator)

**Demonstrates**: Generating labeled images for object detection

**What it does**:
- Creates camera at (2, 0, 1) looking at origin
- Randomizes cube position (-1 to +1 m in X/Y, 0.5-1.5m in Z)
- Randomizes cube rotation (0-360° all axes)
- Randomizes light intensity (1000-5000 lux)
- Captures 1000 frames with 2D bounding box annotations
- Saves to `/workspace/synthetic_data/`

**Output**: 1000 RGB images + JSON labels

---

## Validation and Testing

```bash
# Check Isaac Sim installation
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh --version

# Test ROS 2 bridge
# In Isaac Sim: Window → Extensions → enable omni.isaac.ros2_bridge
# Play simulation
# In terminal:
source /opt/ros/humble/setup.bash
ros2 topic list  # Should show Isaac topics
```

---

## Common Errors

**1. CUDA driver version insufficient**
- Fix: `sudo apt install nvidia-driver-535` (or newer)

**2. Out of memory (VRAM)**
- Cause: GPU has < 8GB, scene too complex
- Fix: Reduce rendering quality (Edit → Preferences → Rendering → Low), use fewer parallel envs

**3. Black screen on launch**
- Fix: Disable Vulkan: `export OMNI_ALLOW_VULKAN=0`

**4. Isaac Sim not found after Omniverse install**
- Fix: Check `~/.local/share/ov/pkg/` for isaac_sim folder, run `./isaac-sim.sh` directly

**5. ROS 2 topics not publishing**
- Cause: ROS2 bridge extension not enabled
- Fix: Window → Extensions → search "ROS2" → enable omni.isaac.ros2_bridge

---

## System Requirements

**Minimum**:
- GPU: NVIDIA GTX 1070 (8GB VRAM) - limited features
- CPU: 4-core x86_64
- RAM: 16GB
- Disk: 50GB
- OS: Ubuntu 20.04/22.04

**Recommended**:
- GPU: NVIDIA RTX 3060 (12GB VRAM) or better
- CPU: 8-core x86_64
- RAM: 32GB
- Disk: 100GB SSD
- OS: Ubuntu 22.04 LTS

**Optimal** (RL training):
- GPU: NVIDIA RTX 4090 (24GB VRAM)
- CPU: 16-core AMD Ryzen/Intel
- RAM: 64GB
- Disk: 500GB NVMe SSD

---

## Related Chapter Content

- Section 1.1: Isaac Ecosystem (Sim, ROS, Lab)
- Section 1.2: Omniverse and USD
- Section 1.3: GPU-Accelerated Simulation
- Section 2.1: Installing Isaac Sim
- Section 2.3: Importing Robots
- Section 2.4: ROS 2 Bridge Setup
- Section 2.5: Creating Parallel Environments
- Section 2.6: Synthetic Data Generation

---

## Cloud Alternative (No Local GPU)

### AWS EC2 g4dn.xlarge

```bash
# Launch instance with Ubuntu 22.04 + NVIDIA T4
# SSH with X11 forwarding
ssh -X ubuntu@<instance-ip>

# Install NVIDIA driver
sudo apt install nvidia-driver-525

# Install Isaac Sim via Docker
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
docker run --gpus all -it nvcr.io/nvidia/isaac-sim:2023.1.1
```

**Cost**: ~$0.50/hour (spot instances cheaper)

---

**Last Updated**: 2025-12-23
**Tested On**: Ubuntu 22.04, Isaac Sim 2023.1.1, RTX 3090, ROS 2 Humble
