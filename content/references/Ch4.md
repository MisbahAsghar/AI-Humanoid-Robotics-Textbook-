# Chapter 4 References: URDF and Robot Modeling

## Foundational Sources (Established)

### 1. Siciliano & Khatib (2016) - Springer Handbook of Robotics
**Citation**: Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer Handbook of Robotics* (2nd ed.). Springer.
**DOI**: [https://doi.org/10.1007/978-3-319-32552-1](https://doi.org/10.1007/978-3-319-32552-1)
**Label**: [established]
**Summary**: Chapters 1-3 cover robot kinematics, coordinate frames, Denavit-Hartenberg parameters, forward/inverse kinematics.
**Relevance**: Theoretical foundation for URDF joint/link modeling and coordinate transformations.

### 2. Craig (2005) - Introduction to Robotics
**Citation**: Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.
**ISBN**: 978-0201543612
**Label**: [established]
**Summary**: Standard robotics textbook covering kinematics, dynamics, inertia tensors, Jacobians.
**Relevance**: Inertia calculations for URDF `<inertial>` tags, joint types, coordinate frames.

### 3. Koenig & Howard (2004) - Gazebo Simulator
**Citation**: Koenig, N., & Howard, A. (2004). "Design and Use Paradigms for Gazebo, An Open-Source Multi-Robot Simulator." *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, 2149-2154.
**DOI**: [https://doi.org/10.1109/IROS.2004.1389727](https://doi.org/10.1109/IROS.2004.1389727)
**Label**: [established]
**Summary**: Original Gazebo paper describing SDF format, physics engines, sensor simulation.
**Relevance**: Background on URDF-to-SDF conversion, Gazebo plugins for sensors.

## Tool Documentation

### 4. URDF Documentation (2025)
**Citation**: ROS 2 Documentation. (2025). *Unified Robot Description Format (URDF) Specification*. [Online].
**URL**: [http://wiki.ros.org/urdf](http://wiki.ros.org/urdf)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: Official URDF XML specification: link, joint, sensor, transmission elements with examples.
**Relevance**: Primary reference for URDF syntax (Sections 1.2-1.4).

### 5. RViz Documentation (2025)
**Citation**: ROS 2 Documentation. (2025). *RViz 2 User Guide*. [Online].
**URL**: [https://github.com/ros2/rviz](https://github.com/ros2/rviz)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: RViz visualization tool for displaying URDF models, TF trees, sensor data.
**Relevance**: URDF visualization examples (Sections 2.2-2.3).

### 6. urdfdom Documentation (2025)
**Citation**: urdfdom Project. (2025). *URDF Parser and Tools*. [Online].
**URL**: [https://github.com/ros/urdfdom](https://github.com/ros/urdfdom)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: C++/Python URDF parser, `check_urdf` validation tool.
**Relevance**: URDF validation (Section 2.5).

### 7. Gazebo Documentation (2025)
**Citation**: Gazebo Project. (2025). *Gazebo Simulator Documentation*. [Online].
**URL**: [https://gazebosim.org/docs](https://gazebosim.org/docs)
**Accessed**: December 2025
**Label**: [tool documentation]
**Summary**: Physics simulation, SDF format, ROS 2 plugins for sensors (camera, LiDAR, IMU).
**Relevance**: Sensor simulation (Section 1.4), Gazebo loading (Section 2.4).

## Emerging Sources

### 8. Universal Robots URDF (2024)
**Citation**: Universal Robots. (2024). *UR5e Robot URDF Description*. [Online].
**URL**: [https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Production URDF for UR5e collaborative robot: 6-DOF arm with accurate inertia, collision geometry.
**Relevance**: Real-world URDF example for industrial manipulator.

### 9. Boston Dynamics Spot URDF (2024)
**Citation**: Boston Dynamics. (2024). *Spot Robot URDF*. [Online].
**URL**: [https://github.com/clearpathrobotics/spot_ros2](https://github.com/clearpathrobotics/spot_ros2)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Spot quadruped URDF: 12-DOF legs, floating base, IMU/camera sensors.
**Relevance**: Complex URDF example for legged robot.

### 10. MoveIt 2 URDF Guidelines (2024)
**Citation**: MoveIt 2 Project. (2024). *URDF Setup for MoveIt*. [Online].
**URL**: [https://moveit.picknik.ai/main/doc/tutorials/urdf_srdf/urdf_srdf_tutorial.html](https://moveit.picknik.ai/main/doc/tutorials/urdf_srdf/urdf_srdf_tutorial.html)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Best practices for URDF in motion planning: collision geometry, self-collision, joint limits.
**Relevance**: URDF optimization for manipulation (collision geometry guidelines).

### 11. Xacro Documentation (2024)
**Citation**: ROS 2 Xacro. (2024). *XML Macros for URDF*. [Online].
**URL**: [http://wiki.ros.org/xacro](http://wiki.ros.org/xacro)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: Xacro preprocessor for URDF: macros, constants, mathematical expressions to reduce repetition.
**Relevance**: Advanced URDF technique for humanoids with symmetric limbs.

### 12. Inertia Calculation Tools (2024)
**Citation**: MeshLab/Blender. (2024). *Mesh Inertia Computation Tools*. [Online].
**URL**: [https://www.meshlab.net/](https://www.meshlab.net/)
**Accessed**: December 2025
**Label**: [emerging]
**Summary**: CAD tools for computing mass properties from 3D meshes (STL, OBJ).
**Relevance**: Inertia tensor calculation for complex geometries (Section 1.2.2).

---

## Additional Resources

### Books
- Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press. (Advanced kinematics/dynamics)

### Tutorials
- ROS 2 URDF Tutorials: [https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)

---

**Note**: All DOIs and URLs verified as of December 2025.
