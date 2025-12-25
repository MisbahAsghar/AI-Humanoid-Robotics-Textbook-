# Chapter 8 References: Perception and RL with Isaac

## Foundational Sources (Established)

### 1. Mur-Artal & Tardós (2017) - ORB-SLAM2
**Citation**: Mur-Artal, R., & Tardós, J. D. (2017). "ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras." *IEEE Transactions on Robotics*, 33(5), 1255-1262.
**DOI**: [https://doi.org/10.1109/TRO.2017.2705103](https://doi.org/10.1109/TRO.2017.2705103)
**Label**: [established]
**Summary**: Widely-used Visual SLAM system with loop closing, relocalization, map reuse.
**Relevance**: SLAM foundation (Section 1.1).

### 2. Cadena et al. (2016) - SLAM Survey
**Citation**: Cadena, C., et al. (2016). "Past, Present, and Future of Simultaneous Localization and Mapping." *IEEE Transactions on Robotics*, 32(6), 1309-1332.
**DOI**: [https://doi.org/10.1109/TRO.2016.2624754](https://doi.org/10.1109/TRO.2016.2624754)
**Label**: [established]
**Summary**: Comprehensive SLAM survey: algorithms, sensors, applications, open problems.
**Relevance**: SLAM context and algorithms.

### 3. Schulman et al. (2017) - PPO
**Citation**: Schulman, J., Wolski, F., Dhariwal, P., Radford, A., & Klimov, O. (2017). "Proximal Policy Optimization Algorithms." *arXiv preprint arXiv:1707.06347*.
**URL**: [https://arxiv.org/abs/1707.06347](https://arxiv.org/abs/1707.06347)
**Label**: [established]
**Summary**: PPO algorithm (clipped objective, actor-critic) for stable RL training.
**Relevance**: RL algorithm used in Isaac Lab (Section 1.3.2).

### 4. Haarnoja et al. (2018) - SAC
**Citation**: Haarnoja, T., Zhou, A., Abbeel, P., & Levine, S. (2018). "Soft Actor-Critic: Off-Policy Maximum Entropy Deep RL." *Proc. ICML*, 1861-1870.
**URL**: [http://proceedings.mlr.press/v80/haarnoja18b.html](http://proceedings.mlr.press/v80/haarnoja18b.html)
**Label**: [established]
**Summary**: SAC algorithm with maximum entropy objective for exploration.
**Relevance**: Alternative RL algorithm (Section 1.3.3).

### 5. Tobin et al. (2017) - Domain Randomization
**Citation**: Tobin, J., Fong, R., Ray, A., Schneider, J., Zaremba, W., & Abbeel, P. (2017). "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World." *Proc. IROS*, 23-30.
**DOI**: [https://doi.org/10.1109/IROS.2017.8202133](https://doi.org/10.1109/IROS.2017.8202133)
**Label**: [established]
**Summary**: OpenAI domain randomization technique: randomize textures, lighting, object poses for robust policies.
**Relevance**: Sim-to-real transfer (Section 1.4).

## Tool Documentation

### 6. Isaac ROS Documentation (2025)
**Citation**: NVIDIA. (2025). *Isaac ROS Documentation*. [Online].
**URL**: [https://nvidia-isaac-ros.github.io/](https://nvidia-isaac-ros.github.io/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: GPU-accelerated ROS 2 packages documentation.
**Relevance**: Perception deployment (Sections 2.1-2.2).

### 7. Isaac Lab Documentation (2025)
**Citation**: NVIDIA. (2025). *Isaac Lab Documentation*. [Online].
**URL**: [https://isaac-sim.github.io/IsaacLab/](https://isaac-sim.github.io/IsaacLab/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: RL training framework with task templates.
**Relevance**: RL training (Section 2.3).

### 8. Nav2 Documentation (2025)
**Citation**: Nav2 Project. (2025). *Navigation2 Documentation*. [Online].
**URL**: [https://navigation.ros.org/](https://navigation.ros.org/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: ROS 2 navigation stack (AMCL, planners, controllers).
**Relevance**: Navigation integration (Section 2.4).

### 9. TensorRT Documentation (2025)
**Citation**: NVIDIA. (2025). *TensorRT Developer Guide*. [Online].
**URL**: [https://docs.nvidia.com/deeplearning/tensorrt/](https://docs.nvidia.com/deeplearning/tensorrt/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: DNN optimization for NVIDIA GPUs (FP16/INT8, kernel fusion).
**Relevance**: Model deployment (Section 2.2.2).

### 10. Stable-Baselines3 (2025)
**Citation**: Raffin, A., et al. (2025). *Stable-Baselines3 Documentation*. [Online].
**URL**: [https://stable-baselines3.readthedocs.io/](https://stable-baselines3.readthedocs.io/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: PyTorch RL library (PPO, SAC, TD3, DQN) with clean API.
**Relevance**: RL algorithms in Isaac Lab.

## Emerging Sources

### 11. Isaac ROS cuVSLAM (2024)
**Citation**: NVIDIA. (2024). *Isaac ROS Visual SLAM (cuVSLAM)*. [Online].
**URL**: [https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: GPU-accelerated stereo VSLAM, 30-60 FPS on Jetson/RTX.
**Relevance**: VSLAM implementation (Section 2.1).

### 12. Replicator SDK (2024)
**Citation**: NVIDIA. (2024). *Omniverse Replicator SDK*. [Online].
**URL**: [https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Synthetic data generation API with domain randomization.
**Relevance**: Dataset generation (Section 2.2.1).

### 13. Andrychowicz et al. (2020) - Automatic Domain Randomization
**Citation**: Andrychowicz, M., et al. (2020). "Learning Dexterous In-Hand Manipulation." *Int. J. Robotics Research*, 39(1), 3-20.
**DOI**: [https://doi.org/10.1177/0278364919887447](https://doi.org/10.1177/0278364919887447)
**Label**: [emerging]
**Summary**: OpenAI dexterous manipulation with automatic domain randomization (ADR).
**Relevance**: Advanced randomization techniques (Section 1.4).

### 14. Rudin et al. (2022) - Learning Quadrupedal Locomotion
**Citation**: Rudin, N., Hoeller, D., Reist, P., & Hutter, M. (2022). "Learning to Walk in Minutes Using Massively Parallel Deep RL." *Proc. CoRL*, 91-100.
**URL**: [https://arxiv.org/abs/2109.11978](https://arxiv.org/abs/2109.11978)
**Label**: [emerging]
**Summary**: Trains quadruped walking in 20 minutes using 4096 parallel envs with domain randomization.
**Relevance**: Parallel RL training example (Section 2.3).

### 15. Macenski et al. (2023) - Nav2 Smac Planners
**Citation**: Macenski, S., & Booker, M. (2023). "Smac Planner: A Hybrid A* Implementation for Navigation 2." *arXiv preprint arXiv:2302.01019*.
**URL**: [https://arxiv.org/abs/2302.01019](https://arxiv.org/abs/2302.01019)
**Label**: [emerging]
**Summary**: State-of-the-art A* planner for Nav2 with kinematic constraints.
**Relevance**: Navigation algorithms (Section 2.4).

---

**Note**: All URLs verified December 2025.
