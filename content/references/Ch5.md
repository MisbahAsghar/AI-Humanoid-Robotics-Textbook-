# Chapter 5 References: Gazebo Simulation

## Foundational Sources (Established)

### 1. Koenig & Howard (2004) - Gazebo Simulator
**Citation**: Koenig, N., & Howard, A. (2004). "Design and Use Paradigms for Gazebo, An Open-Source Multi-Robot Simulator." *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, 2149-2154.
**DOI**: [https://doi.org/10.1109/IROS.2004.1389727](https://doi.org/10.1109/IROS.2004.1389727)
**Label**: [established]
**Summary**: Original Gazebo paper describing architecture, SDF format, OGRE rendering, ODE physics integration.
**Relevance**: Foundation for understanding Gazebo design and capabilities.

### 2. Smith (2006) - ODE Physics Engine
**Citation**: Smith, R. (2006). *Open Dynamics Engine User Guide*. [Online].
**URL**: [http://www.ode.org/](http://www.ode.org/)
**Label**: [established]
**Summary**: Documentation for ODE (Open Dynamics Engine), default physics engine in Gazebo Classic.
**Relevance**: Physics simulation principles (Section 1.2).

### 3. Coumans (2015) - Bullet Physics
**Citation**: Coumans, E., & Bai, Y. (2015). *Bullet Physics SDK Manual*. [Online].
**URL**: [https://pybullet.org/](https://pybullet.org/)
**Label**: [established]
**Summary**: Bullet physics engine for collision detection, rigid body dynamics, constraints.
**Relevance**: Alternative physics engine for Gazebo.

### 4. Lee et al. (2018) - DART
**Citation**: Lee, J., Grey, M. X., Ha, S., Kunz, T., Jain, S., Ye, Y., Srinivasa, S. S., Stilman, M., & Liu, C. K. (2018). "DART: Dynamic Animation and Robotics Toolkit." *Journal of Open Source Software*, 3(22), 500.
**DOI**: [https://doi.org/10.21105/joss.00500](https://doi.org/10.21105/joss.00500)
**Label**: [established]
**Summary**: DART physics engine with inverse dynamics, soft body support, analytical gradients.
**Relevance**: Advanced physics for humanoid simulation.

## Tool Documentation

### 5. Gazebo Classic Documentation (2025)
**Citation**: Gazebo Project. (2025). *Gazebo Classic 11 Documentation*. [Online].
**URL**: [http://classic.gazebosim.org/](http://classic.gazebosim.org/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: Official docs for Gazebo Classic: tutorials, SDF specification, plugin API, model database.
**Relevance**: Primary reference for all Gazebo usage (Sections 1-2).

### 6. SDF Specification (2025)
**Citation**: Gazebo Project. (2025). *SDF (Simulation Description Format) Specification*. [Online].
**URL**: [http://sdformat.org/](http://sdformat.org/)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: XML schema for SDF 1.6/1.7/1.8: world, model, link, joint, sensor, plugin elements.
**Relevance**: SDF world file syntax (Section 2.2).

### 7. gazebo_ros_pkgs Documentation (2025)
**Citation**: ROS 2 Gazebo. (2025). *gazebo_ros_pkgs for ROS 2*. [Online].
**URL**: [https://github.com/ros-simulation/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: ROS 2 integration packages for Gazebo: spawn_entity, ROS plugins (camera, LiDAR, IMU, differential drive).
**Relevance**: ROS 2 bridge for Gazebo (Sections 2.3-2.4).

### 8. RViz Documentation (2025)
**Citation**: ROS 2 Documentation. (2025). *RViz 2 User Guide*. [Online].
**URL**: [https://github.com/ros2/rviz](https://github.com/ros2/rviz)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: 3D visualization for sensor data (LaserScan, Image, PointCloud2).
**Relevance**: Visualizing Gazebo sensor outputs (Section 2.4).

## Emerging Sources

### 9. Gazebo Sim (2024)
**Citation**: Gazebo Project. (2024). *Gazebo Simulator (Ignition → Gazebo)*. [Online].
**URL**: [https://gazebosim.org/](https://gazebosim.org/)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Next-generation Gazebo with OGRE 2 rendering, multi-threaded physics, GPU sensors, Vulkan support.
**Relevance**: Future alternative to Gazebo Classic (EOL Jan 2025).

### 10. ros_gz (2024)
**Citation**: ROS 2 Gazebo Sim Integration. (2024). *ros_gz Bridge*. [Online].
**URL**: [https://github.com/gazebosim/ros_gz](https://github.com/gazebosim/ros_gz)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: ROS 2 bridge for Gazebo Sim (replaces gazebo_ros_pkgs).
**Relevance**: Migration path from Gazebo Classic to Gazebo Sim.

### 11. Domain Randomization (2024)
**Citation**: Tobin, J., et al. (2017). "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World." *Proc. IEEE/RSJ IROS*, 23-30.
**DOI**: [https://doi.org/10.1109/IROS.2017.8202133](https://doi.org/10.1109/IROS.2017.8202133)
**Label**: [established]
**Summary**: OpenAI technique: randomize simulation parameters (physics, textures, lighting) to train robust policies.
**Relevance**: Sim-to-real transfer strategies (Section 3.1).

### 12. Gazebo Model Database (2024)
**Citation**: Gazebo Project. (2024). *Gazebo Model Database*. [Online].
**URL**: [https://github.com/osrf/gazebo_models](https://github.com/osrf/gazebo_models)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Repository of pre-built models: furniture, buildings, robots for quick world creation.
**Relevance**: Reusable assets for simulation environments.

### 13. MuJoCo Simulator (2024)
**Citation**: Todorov, E., Erez, T., & Tassa, Y. (2012). "MuJoCo: A Physics Engine for Model-Based Control." *Proc. IROS*, 5026-5033.
**DOI**: [https://doi.org/10.1109/IROS.2012.6386109](https://doi.org/10.1109/IROS.2012.6386109)
**Label**: [established]
**Summary**: Alternative physics simulator optimized for contact-rich tasks, RL (now open-source).
**Relevance**: Comparison to Gazebo for humanoid simulation.

### 14. PyBullet (2024)
**Citation**: Coumans, E., & Bai, Y. (2016-2024). *PyBullet Physics Simulation*. [Online].
**URL**: [https://pybullet.org/](https://pybullet.org/)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Python interface to Bullet physics, popular for RL (OpenAI Gym, stable-baselines3).
**Relevance**: Alternative to Gazebo for machine learning research.

### 15. NVIDIA Isaac Sim (2024)
**Citation**: NVIDIA. (2024). *Isaac Sim Documentation*. [Online].
**URL**: [https://docs.omniverse.nvidia.com/isaacsim/](https://docs.omniverse.nvidia.com/isaacsim/)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: GPU-accelerated simulation on Omniverse (covered in Chapters 7-8).
**Relevance**: Alternative simulator with photorealistic rendering, domain randomization.

---

## Additional Resources

### Tutorials
- Gazebo Tutorials: [http://classic.gazebosim.org/tutorials](http://classic.gazebosim.org/tutorials)
- ROS 2 + Gazebo Integration: [https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)

---

**Note**: All URLs verified December 2025. Gazebo Classic EOL January 2025; migrate to Gazebo Sim for long-term projects.
