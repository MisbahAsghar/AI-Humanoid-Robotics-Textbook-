# Chapter 7 References: Isaac Platform Overview

## Foundational Sources (Established)

### 1. Makoviychuk et al. (2021) - Isaac Gym
**Citation**: Makoviychuk, V., Wawrzyniak, L., Guo, Y., Lu, M., Storey, K., Macklin, M., Hoeller, D., Rudin, N., Allshire, A., Handa, A., & State, G. (2021). "Isaac Gym: High Performance GPU-Based Physics Simulation for Robot Learning." *arXiv preprint arXiv:2108.10470*.
**URL**: [https://arxiv.org/abs/2108.10470](https://arxiv.org/abs/2108.10470)
**Label**: [established]
**Summary**: Original Isaac Gym paper demonstrating GPU-accelerated parallel RL training (16,384 environments on 8 GPUs), 100-1000× speedup over CPU simulators, humanoid locomotion trained in hours vs days.
**Relevance**: Foundation for Isaac Lab framework (Section 1.1.3).

### 2. Todorov, Erez & Tassa (2012) - MuJoCo
**Citation**: Todorov, E., Erez, T., & Tassa, Y. (2012). "MuJoCo: A Physics Engine for Model-Based Control." *Proc. IROS*, 5026-5033.
**DOI**: [https://doi.org/10.1109/IROS.2012.6386109](https://doi.org/10.1109/IROS.2012.6386109)
**Label**: [established]
**Summary**: Physics engine optimized for contact-rich control, used in RL research (comparison baseline for Isaac).
**Relevance**: Alternative physics simulator, comparison to PhysX.

### 3. Macklin et al. (2014) - Position Based Dynamics
**Citation**: Macklin, M., Müller, M., & Chentanez, N. (2016). "XPBD: Position-Based Simulation of Compliant Constrained Dynamics." *Proc. Motion in Games*, 49-54.
**DOI**: [https://doi.org/10.1145/2994258.2994272](https://doi.org/10.1145/2994258.2994272)
**Label**: [established]
**Summary**: Constraint-based physics algorithm used in PhysX 5 for stable simulations.
**Relevance**: Physics foundation for Isaac Sim (Section 1.3).

## Tool Documentation

### 4. Isaac Sim Documentation (2025)
**Citation**: NVIDIA. (2025). *Isaac Sim Documentation*. [Online].
**URL**: [https://docs.omniverse.nvidia.com/isaacsim/latest/](https://docs.omniverse.nvidia.com/isaacsim/latest/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: Official Isaac Sim docs: installation, tutorials, Python API, ROS 2 bridge, sensor configuration.
**Relevance**: Primary reference for all Isaac Sim usage.

### 5. Isaac ROS Documentation (2025)
**Citation**: NVIDIA. (2025). *Isaac ROS Documentation*. [Online].
**URL**: [https://nvidia-isaac-ros.github.io/](https://nvidia-isaac-ros.github.io/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: GPU-accelerated ROS 2 packages: image pipeline, DNN inference, Visual SLAM, nvblox, AprilTag detection.
**Relevance**: Isaac ROS deployment (Section 1.1.2, Section 3.1).

### 6. Isaac Lab Documentation (2025)
**Citation**: NVIDIA. (2025). *Isaac Lab (formerly OmniIsaacGymEnvs)*. [Online].
**URL**: [https://isaac-sim.github.io/IsaacLab/](https://isaac-sim.github.io/IsaacLab/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: RL training framework: pre-built tasks, parallel environments, stable-baselines3/RLlib integration.
**Relevance**: RL training workflow (Section 1.1.3, Section 2.5).

### 7. USD Documentation (2025)
**Citation**: Pixar. (2025). *Universal Scene Description (USD) Documentation*. [Online].
**URL**: [https://graphics.pixar.com/usd/docs/](https://graphics.pixar.com/usd/docs/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: USD format specification: prims, attributes, layers, composition, schemas.
**Relevance**: Understanding USD format (Section 1.2.2).

### 8. Omniverse Documentation (2025)
**Citation**: NVIDIA. (2025). *NVIDIA Omniverse Platform Documentation*. [Online].
**URL**: [https://docs.omniverse.nvidia.com/](https://docs.omniverse.nvidia.com/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: Omniverse platform: Nucleus, Connectors, Create, View, collaboration workflows.
**Relevance**: Omniverse platform overview (Section 1.2.1).

## Emerging Sources

### 9. Isaac Sim 2023.1.1 Release (2024)
**Citation**: NVIDIA. (2024). *Isaac Sim 2023.1.1 Release Notes*. [Online].
**URL**: [https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html](https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Latest Isaac Sim features: improved ROS 2 Humble support, Isaac Lab integration, warehouse example scenes.
**Relevance**: Current version capabilities (Section 2.1).

### 10. NVIDIA Jetson AGX Orin (2024)
**Citation**: NVIDIA. (2024). *Jetson AGX Orin Developer Kit*. [Online].
**URL**: [https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Embedded GPU (275 TOPS AI, 32GB RAM) for Isaac ROS deployment, runs Isaac ROS packages at 30-60 FPS.
**Relevance**: Target hardware for Isaac ROS (Section 3.1).

### 11. Replicator Documentation (2024)
**Citation**: NVIDIA. (2024). *Omniverse Replicator SDK*. [Online].
**URL**: [https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Synthetic data generation API: domain randomization, scene composition, annotation writers.
**Relevance**: Synthetic dataset generation (Section 2.6).

### 12. PhysX 5 Documentation (2024)
**Citation**: NVIDIA. (2024). *PhysX 5 SDK Documentation*. [Online].
**URL**: [https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/](https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: GPU physics engine: rigid bodies, soft bodies, articulations, scene queries, contact modification.
**Relevance**: Physics simulation in Isaac Sim (Section 1.3).

### 13. Isaac Manipulator Examples (2024)
**Citation**: NVIDIA. (2024). *Isaac Sim Manipulation Examples*. [Online].
**URL**: [https://github.com/NVIDIA-Omniverse/IsaacSim-Manipulation-Examples](https://github.com/NVIDIA-Omniverse/IsaacSim-Manipulation-Examples)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Example projects: pick-and-place, bin picking, assembly tasks with Franka/UR robots.
**Relevance**: Practical Isaac Sim use cases.

### 14. RTX Rendering (2024)
**Citation**: NVIDIA. (2024). *RTX Real-Time Ray Tracing Technology*. [Online].
**URL**: [https://www.nvidia.com/en-us/geforce/technologies/rtx/](https://www.nvidia.com/en-us/geforce/technologies/rtx/)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: RT cores for hardware-accelerated ray tracing, OptiX denoising, DLSS upscaling.
**Relevance**: Photorealistic rendering in Isaac Sim (Section 1.4).

### 15. ROS 2 Isaac Workflows (2024)
**Citation**: NVIDIA. (2024). *ROS 2 Workflows with Isaac Sim*. [Online].
**URL**: [https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Tutorials for ROS 2 integration: joint control, sensor publishing, Nav2 integration, MoveIt integration.
**Relevance**: ROS 2 bridge setup (Section 2.4).

---

## Additional Resources

### Community
- NVIDIA Isaac Forums: [https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/](https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/)
- Isaac Sim GitHub Discussions: [https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles/discussions](https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles/discussions)

### Video Tutorials
- NVIDIA Isaac Sim YouTube: [https://www.youtube.com/c/NVIDIAOmniverse](https://www.youtube.com/c/NVIDIAOmniverse)

---

**Note**: All URLs verified December 2025. Isaac Sim requires NVIDIA RTX GPU; AMD/Intel GPUs not supported.
