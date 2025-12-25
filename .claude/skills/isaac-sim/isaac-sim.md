# Isaac Sim

## Purpose
Guide NVIDIA Isaac Sim setup for GPU-accelerated robot simulation, including USD scene creation, synthetic data generation, and parallel environment training.

## When to Use
- Training at scale with thousands of parallel environments
- Generating synthetic datasets for perception models
- Using RTX ray-tracing for photorealistic rendering
- Integrating ROS 2 with Isaac Sim environments
- Simulating complex warehouse or factory scenarios

## Book Chapters
- Chapter 7: Isaac Sim and NVIDIA Omniverse

## Example Usage
```
User: "Set up domain randomization in Isaac Sim for robustness training"

Claude: Using the isaac-sim skill, Chapter 7 Section 1.4 covers randomization:

Physics randomization:
```python
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim

# Randomize object mass
mass = np.random.uniform(0.8, 1.2) * nominal_mass
prim.GetAttribute("physics:mass").Set(mass)

# Randomize friction
friction = np.random.uniform(0.5, 1.5)
prim.GetAttribute("physics:dynamicFriction").Set(friction)
```

Visual randomization:
- Lighting: randomize intensity, color temperature
- Textures: swap from 1000+ material library

See Chapter 7, Section 1.4 for camera and force randomization.
```
