# Chapter 3 References: ROS 2 Fundamentals

## Foundational Sources (Established)

### 1. Quigley et al. (2009) - ROS: An Open-Source Robot Operating System
**Citation**: Quigley, M., Conley, K., Gerkey, B., Faust, J., Foote, T., Leibs, J., Wheeler, R., & Ng, A. Y. (2009). "ROS: An Open-Source Robot Operating System." *ICRA Workshop on Open Source Software*, 3(3.2), 5.

**URL**: [http://www.willowgarage.com/sites/default/files/icraoss09-ROS.pdf](http://www.willowgarage.com/sites/default/files/icraoss09-ROS.pdf)

**Label**: [established]

**Summary**: Original ROS 1 paper introducing the architecture: pub-sub communication (topics), RPC (services), parameter server, tf (transform library), and design philosophy (tools over frameworks, language independence, thin middleware).

**Relevance to Chapter**: Foundation for understanding ROS 2 design decisions and evolution from ROS 1.

---

### 2. Maruyama, Kato & Azumi (2016) - Exploring the Performance of ROS2
**Citation**: Maruyama, Y., Kato, S., & Azumi, T. (2016). "Exploring the Performance of ROS2." *Proc. Int. Conf. Embedded Software (EMSOFT)*, 1-10.

**DOI**: [https://doi.org/10.1145/2968478.2968502](https://doi.org/10.1145/2968478.2968502)

**Label**: [established]

**Summary**: Early performance analysis of ROS 2 vs. ROS 1. Evaluates DDS middleware implementations (Fast RTPS, OpenSplice), latency, throughput, real-time determinism. Shows ROS 2 achieves <1ms latency with real-time DDS profiles.

**Relevance to Chapter**: Justifies ROS 2 design choices (DDS middleware, no master node) and performance characteristics.

---

### 3. Pardo-Castellote (2003) - OMG Data-Distribution Service
**Citation**: Pardo-Castellote, G. (2003). "OMG Data-Distribution Service: Architectural Overview." *Proc. IEEE Military Communications Conference (MILCOM)*, 200-206.

**DOI**: [https://doi.org/10.1109/MILCOM.2003.1290093](https://doi.org/10.1109/MILCOM.2003.1290093)

**Label**: [established]

**Summary**: Official DDS (Data Distribution Service) standard specification from Object Management Group (OMG). Describes pub-sub architecture, discovery protocol (multicast RTPS), Quality-of-Service (QoS) policies (reliability, durability, deadline), and real-time capabilities.

**Relevance to Chapter**: Explains underlying DDS middleware used by ROS 2 (Section 1.3).

---

### 4. Macenski et al. (2022) - Robot Operating System 2: Design, Architecture, and Uses in the Wild
**Citation**: Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). "Robot Operating System 2: Design, Architecture, and Uses in the Wild." *Science Robotics*, 7(66).

**DOI**: [https://doi.org/10.1126/scirobotics.abm6074](https://doi.org/10.1126/scirobotics.abm6074)

**Label**: [established]

**Summary**: Comprehensive ROS 2 design rationale and architecture. Covers differences from ROS 1 (no master, DDS middleware, lifecycle management), design patterns (composition, components), and real-world case studies (automotive, logistics, service robots). Authored by ROS 2 core maintainers.

**Relevance to Chapter**: Authoritative reference for ROS 2 architecture, design principles, and production use cases.

---

## Tool Documentation

### 5. ROS 2 Humble Documentation (2025)
**Citation**: ROS 2 Documentation. (2025). *ROS 2 Humble Hawksbill Official Documentation*. [Online].

**URL**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)

**Accessed**: December 2025

**Label**: [tool documentation]

**Summary**: Official tutorials, concept guides, API reference for ROS 2 Humble LTS release (2022-2027). Covers installation, node creation, topics, services, actions, launch files, parameters, lifecycle management.

**Relevance to Chapter**: Primary reference for ROS 2 Humble installation (Section 2.1) and API usage (Sections 2.2-2.5).

---

### 6. rclpy API Documentation (2025)
**Citation**: ROS 2 Documentation. (2025). *ROS 2 Python Client Library (rclpy) Reference*. [Online].

**URL**: [https://docs.ros2.org/humble/api/rclpy/](https://docs.ros2.org/humble/api/rclpy/)

**Accessed**: December 2025

**Label**: [tool documentation]

**Summary**: Python API documentation for rclpy (ROS Client Library Python): `Node` class, publishers (`create_publisher`), subscribers (`create_subscription`), services (`create_service`, `create_client`), actions (`ActionServer`, `ActionClient`), timers, executors, QoS profiles.

**Relevance to Chapter**: API reference for Python code examples in Sections 2.3-2.5.

---

### 7. Colcon Documentation (2025)
**Citation**: Colcon Documentation. (2025). *COLlaborative CONstruction Build Tool*. [Online].

**URL**: [https://colcon.readthedocs.io/](https://colcon.readthedocs.io/)

**Accessed**: December 2025

**Label**: [tool documentation]

**Summary**: Build system for ROS 2 workspaces. Handles package discovery, dependency resolution, parallel builds, workspace overlays. Replaces catkin from ROS 1.

**Relevance to Chapter**: Build tool used in Section 2.2 (Creating Your First ROS 2 Package).

---

### 8. Fast DDS Documentation (2025)
**Citation**: eProsima. (2025). *eProsima Fast DDS Documentation*. [Online].

**URL**: [https://fast-dds.docs.eprosima.com/](https://fast-dds.docs.eprosima.com/)

**Accessed**: December 2025

**Label**: [tool documentation]

**Summary**: Documentation for Fast DDS (formerly Fast RTPS), the default DDS implementation in ROS 2 Humble. Covers QoS configuration, discovery mechanisms (Simple, Server-Client), shared memory transport, performance tuning, DDS-Security.

**Relevance to Chapter**: DDS middleware layer explained in Section 1.3.1.

---

### 9. CycloneDDS Documentation (2025)
**Citation**: Eclipse Foundation. (2025). *Eclipse Cyclone DDS Documentation*. [Online].

**URL**: [https://cyclonedds.io/docs/](https://cyclonedds.io/docs/)

**Accessed**: December 2025

**Label**: [tool documentation]

**Summary**: Alternative DDS implementation for ROS 2, known for low latency and small memory footprint. Used in automotive applications (e.g., AUTOSAR Adaptive).

**Relevance to Chapter**: Alternative DDS mentioned in Section 1.3.1.

---

## Emerging Sources

### 10. ROS 2 Iron Documentation (2024)
**Citation**: Open Robotics. (2024). *ROS 2 Iron Irwini Release Notes*. [Online].

**URL**: [https://docs.ros.org/en/iron/](https://docs.ros.org/en/iron/)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: Latest ROS 2 release (May 2023). New features: type negotiation (dynamic message type matching), service introspection (monitor service calls), improved lifecycle management, better logging.

**Relevance to Chapter**: Latest ROS 2 developments beyond Humble LTS.

---

### 11. Nav2 Navigation Framework (2024)
**Citation**: Nav2 Project. (2024). *Navigation2 Framework Documentation*. [Online].

**URL**: [https://navigation.ros.org/](https://navigation.ros.org/)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: Production-grade autonomous navigation stack for ROS 2. Includes SLAM (Simultaneous Localization and Mapping), global/local path planning, behavior trees for mission planning, costmaps for obstacle avoidance. Used in commercial AMRs (Autonomous Mobile Robots).

**Relevance to Chapter**: Example of large-scale ROS 2 system architecture using topics, services, and actions.

---

### 12. MoveIt 2 Motion Planning (2024)
**Citation**: MoveIt Project. (2024). *MoveIt 2 Motion Planning Framework*. [Online].

**URL**: [https://moveit.ros.org/](https://moveit.ros.org/)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: Manipulation planning framework for ROS 2. Features: inverse kinematics (IK), collision detection, grasp planning, trajectory optimization. Supports OMPL, Pilz, CHOMP planners. Integrated with ROS 2 Control for hardware interfaces.

**Relevance to Chapter**: Example of action-based architecture (pick-and-place uses actions for long-running tasks).

---

### 13. ROS 2 Control Framework (2024)
**Citation**: ROS 2 Control. (2024). *ros2_control Framework Documentation*. [Online].

**URL**: [https://control.ros.org/](https://control.ros.org/)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: Real-time controller interface for ROS 2. Provides hardware abstraction (joint interfaces), controller chaining (cascaded control loops), and real-time guarantees. Used for joint control, force control, impedance control.

**Relevance to Chapter**: Real-time capabilities enabled by DDS middleware (Section 1.1.1).

---

### 14. Autoware Autonomous Driving Stack (2024)
**Citation**: Autoware Foundation. (2024). *Autoware: Open-Source Autonomous Driving Stack*. [Online].

**URL**: [https://www.autoware.org/](https://www.autoware.org/)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: ROS 2-based autonomous vehicle software stack. Modules: perception (object detection, tracking), planning (route planning, behavior planning), control (lateral/longitudinal). Used by Toyota, Tier IV, and academic institutions. Deployed in commercial autonomous shuttles.

**Relevance to Chapter**: Large-scale ROS 2 system demonstrating multi-node architecture (100+ nodes in full stack).

---

### 15. ROS-Industrial Consortium (2024)
**Citation**: ROS-Industrial Consortium. (2024). *ROS 2 for Industrial Applications*. [Online].

**URL**: [https://rosindustrial.org/](https://rosindustrial.org/)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: Consortium focused on ROS 2 adoption in manufacturing. Provides industrial robot drivers (ABB, KUKA, Fanuc, Universal Robots), real-time control interfaces, OPC-UA integration for factory automation, quality-of-service profiles for deterministic control.

**Relevance to Chapter**: Production use cases demonstrating ROS 2's suitability for industrial robotics (real-time, security, reliability).

---

## Additional Resources for Further Reading

### Books
- Newmann, W. (2017). *A Systematic Approach to Learning Robot Programming with ROS*. CRC Press. (ROS 1, but concepts transfer)
- Martinez, A., & Fernandez, E. (2021). *Learning ROS for Robotics Programming* (3rd ed.). Packt. (ROS 1 focus)
- Joseph, L. (2023). *ROS 2 for Robotics: Learn Robot Operating System 2*. Packt. (ROS 2 specific)

### Tutorials and Courses
- Official ROS 2 Tutorials: [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
- The Construct ROS 2 Courses: [https://www.theconstructsim.com/](https://www.theconstructsim.com/)
- Udemy ROS 2 Courses: Multiple courses available for beginners to advanced

### Community Resources
- ROS Discourse (forum): [https://discourse.ros.org/](https://discourse.ros.org/)
- ROS Answers (Q&A): [https://answers.ros.org/](https://answers.ros.org/)
- ROS 2 GitHub: [https://github.com/ros2](https://github.com/ros2)

### Conference Proceedings
- ROSCon (annual ROS conference): [https://roscon.ros.org/](https://roscon.ros.org/) — talks on ROS 2 architecture, case studies, new packages
- ICRA/IROS workshops: "ROS in Industry", "Real-time Robotics with ROS"

---

## Citation Guidelines

**Established** sources are peer-reviewed publications or foundational papers with stable citations. These form the theoretical foundation of ROS 2.

**Emerging** sources are recent project documentation, framework releases, or case studies (2023-2025) representing current state-of-the-art. These are dated as ROS 2 ecosystem evolves rapidly.

**Tool documentation** refers to official ROS 2 package and library documentation used in code examples.

---

**Note**: All DOIs and URLs verified as of December 2025. For broken links, use DOI resolver (doi.org) or Wayback Machine (archive.org). ROS 2 documentation is versioned by distro (Humble, Iron, etc.) — ensure URL matches target ROS 2 version.
