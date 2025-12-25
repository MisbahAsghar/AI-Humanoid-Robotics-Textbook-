# Chapter 8 Code Examples

**Chapter**: Perception and RL with Isaac
**Purpose**: GPU-accelerated perception, RL training, domain randomization, sim-to-real

---

## Requirements

- Isaac Sim 2023.1.1+
- NVIDIA RTX 3060+ (for RL training)
- ROS 2 Humble
- Isaac ROS packages (optional)
- PyTorch, stable-baselines3

---

## Example Scripts

### `synthetic_detection_data.py`
**Demonstrates**: Generating labeled object detection datasets with Replicator

**What it does**:
- Creates scene: camera, table, 5 randomized objects (cube/sphere/cylinder)
- Randomizes: object positions (on table), rotations, lighting (500-3000 lux), textures
- Captures 1000 frames with annotations (RGB, 2D bbox, semantic/instance segmentation)
- Outputs to `/workspace/detection_dataset/`

**Running**:
```bash
python synthetic_detection_data.py
```

**Expected**: 1000 images + COCO JSON labels + segmentation masks (~5-10 minutes on RTX 4090)

---

### `train_reach_task.py`
**Demonstrates**: Training RL policy for Franka arm reaching task

**What it does**:
- Creates 512 parallel Franka arm environments
- Task: Reach 3D target position with end-effector
- Uses PPO algorithm (stable-baselines3)
- Trains for 1M steps (~30 minutes on RTX 4090)
- Saves policy to `logs/`

**Running**:
```bash
# Requires Isaac Lab
python train_reach_task.py

# Monitor training
tensorboard --logdir=logs/
```

**Expected**: Success rate increases from ~20% to 95%+ over training

---

## Installation

### Isaac ROS (for perception)

```bash
# Create workspace
mkdir -p ~/isaac_ros_ws/src && cd ~/isaac_ros_ws/src

# Clone packages
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git

# Build
cd ~/isaac_ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### Isaac Lab (for RL)

```bash
# Isaac Lab comes with Isaac Sim 2023.1.1+
# Or install separately:
pip install omni-isaac-lab
```

---

## Common Errors

**1. CUDA out of memory during RL training**
- Fix: Reduce `num_envs` (512 → 256), reduce image resolution

**2. Replicator dataset incomplete**
- Cause: Simulation stopped early
- Fix: Check Isaac Sim console for errors, ensure output_dir writable

**3. Isaac ROS packages not found**
- Fix: Source workspace: `source ~/isaac_ros_ws/install/setup.bash`

**4. TensorRT engine incompatible**
- Cause: Engine built for different GPU architecture
- Fix: Rebuild on target GPU: `trtexec --onnx=model.onnx --fp16`

---

## Related Chapter Content

- Section 2.1: Isaac ROS Visual SLAM Setup
- Section 2.2: Object Detection with Synthetic Data
- Section 2.3: RL with Isaac Lab
- Section 2.4: Nav2 Integration
- Section 3.1: Deploying RL Policy to Real Robot

---

**Last Updated**: 2025-12-23
