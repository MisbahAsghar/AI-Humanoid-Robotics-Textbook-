# Chapter 2 References: Humanoid Sensor Systems

## Foundational Sources (Established)

### 1. Siciliano & Khatib (2016) - Robotics Handbook
**Citation**: Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer Handbook of Robotics* (2nd ed.). Springer.

**DOI**: [https://doi.org/10.1007/978-3-319-32552-1](https://doi.org/10.1007/978-3-319-32552-1)

**Label**: [established]

**Summary**: Comprehensive robotics reference covering sensor types, characteristics, and applications. Part B (Chapters 6-9) covers vision sensors, range sensors, force/torque sensors, and tactile sensing.

**Relevance to Chapter**: Authoritative source for sensor specifications, calibration methods, and multi-sensor integration principles.

---

### 2. Thrun, Burgard & Fox (2005) - Probabilistic Robotics
**Citation**: Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

**ISBN**: 978-0262201629

**Label**: [established]

**Summary**: Standard textbook on probabilistic approaches to robotics. Chapters 6-7 cover sensor models (beam models for LiDAR, landmark detection for cameras), measurement noise, and uncertainty quantification.

**Relevance to Chapter**: Foundation for sensor noise models, Kalman filtering, and sensor fusion algorithms.

---

### 3. Zhang (2000) - Camera Calibration
**Citation**: Zhang, Z. (2000). "A Flexible New Technique for Camera Calibration." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 22(11), 1330-1334.

**DOI**: [https://doi.org/10.1109/34.888718](https://doi.org/10.1109/34.888718)

**Label**: [established]

**Summary**: Widely-adopted method for camera intrinsic calibration using planar checkerboard targets. Estimates focal length, principal point, and radial/tangential distortion coefficients.

**Relevance to Chapter**: Standard calibration procedure implemented in OpenCV `cv2.calibrateCamera()`.

---

### 4. Furgale, Rehder & Siegwart (2013) - Multi-Sensor Calibration
**Citation**: Furgale, P., Rehder, J., & Siegwart, R. (2013). "Unified Temporal and Spatial Calibration for Multi-Sensor Systems." *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, 1280-1286.

**DOI**: [https://doi.org/10.1109/IROS.2013.6696514](https://doi.org/10.1109/IROS.2013.6696514)

**Label**: [established]

**Summary**: Extrinsic calibration method for multi-sensor robotic systems (camera-IMU-LiDAR). Addresses spatial transformations and temporal synchronization between sensors.

**Relevance to Chapter**: Explains coordinate frame transformations and sensor-to-robot calibration (Section 1.4).

---

### 5. Madgwick, Harrison & Vaidyanathan (2011) - IMU Orientation Estimation
**Citation**: Madgwick, S. O. H., Harrison, A. J. L., & Vaidyanathan, R. (2011). "Estimation of IMU and MARG Orientation Using a Gradient Descent Algorithm." *Proc. IEEE Int. Conf. Rehabilitation Robotics*, 1-7.

**DOI**: [https://doi.org/10.1109/ICORR.2011.5975346](https://doi.org/10.1109/ICORR.2011.5975346)

**Label**: [established]

**Summary**: Computationally efficient complementary filter for IMU orientation estimation. Fuses accelerometer, gyroscope, and magnetometer (MARG: Magnetic, Angular Rate, Gravity).

**Relevance to Chapter**: Widely-used open-source algorithm for sensor fusion (Section 1.3.1, code example in Section 2.4).

---

### 6. Endres et al. (2014) - RGBD-SLAM
**Citation**: Endres, F., Hess, J., Sturm, J., Cremers, D., & Burgard, W. (2014). "3-D Mapping with an RGB-D Camera." *IEEE Transactions on Robotics*, 30(1), 177-187.

**DOI**: [https://doi.org/10.1109/TRO.2013.2279412](https://doi.org/10.1109/TRO.2013.2279412)

**Label**: [established]

**Summary**: RGBD-SLAM algorithm using depth cameras (Kinect, RealSense) for simultaneous localization and mapping. Addresses loop closure detection and pose graph optimization.

**Relevance to Chapter**: Application of depth cameras for 3D mapping (Section 1.1.2).

---

### 7. Rusu & Cousins (2011) - Point Cloud Library (PCL)
**Citation**: Rusu, R. B., & Cousins, S. (2011). "3D is Here: Point Cloud Library (PCL)." *Proc. IEEE Int. Conf. Robotics and Automation (ICRA)*, 1-4.

**DOI**: [https://doi.org/10.1109/ICRA.2011.5980567](https://doi.org/10.1109/ICRA.2011.5980567)

**Label**: [established]

**Summary**: Introduction to Point Cloud Library (PCL), an open-source C++ library for 3D point cloud processing. Includes algorithms for filtering, segmentation, feature extraction, and registration.

**Relevance to Chapter**: Standard library for LiDAR and depth camera data processing (Section 2.3).

---

## Tool Documentation

### 8. OpenCV Developers (2025) - Camera Calibration API
**Citation**: OpenCV Developers. (2025). *OpenCV Documentation: Camera Calibration and 3D Reconstruction*. [Online].

**URL**: [https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)

**Accessed**: December 2025

**Label**: [tool documentation]

**Summary**: Official OpenCV API reference for camera calibration (`cv2.calibrateCamera`), stereo vision, pose estimation, and 3D reconstruction.

**Relevance to Chapter**: Implementation guide for camera calibration and image processing (Section 2.2).

---

### 9. Open3D Developers (2025) - 3D Geometry Processing
**Citation**: Open3D Developers. (2025). *Open3D Documentation: Geometry Processing*. [Online].

**URL**: [http://www.open3d.org/docs/release/](http://www.open3d.org/docs/release/)

**Accessed**: December 2025

**Label**: [tool documentation]

**Summary**: Python library for 3D data processing. Provides point cloud I/O, visualization, filtering (voxel downsampling, statistical outlier removal), surface reconstruction, and registration (ICP).

**Relevance to Chapter**: Point cloud visualization and processing (Section 2.3).

---

### 10. ROS 2 Documentation (2025) - sensor_msgs Interface
**Citation**: ROS 2 Documentation. (2025). *sensor_msgs Interface Definitions*. [Online].

**URL**: [https://docs.ros.org/en/humble/p/sensor_msgs/](https://docs.ros.org/en/humble/p/sensor_msgs/)

**Accessed**: December 2025

**Label**: [tool documentation]

**Summary**: Standard ROS 2 message types for sensors: `Image` (cameras), `LaserScan` and `PointCloud2` (LiDAR), `Imu` (IMU), `CameraInfo` (calibration).

**Relevance to Chapter**: Standardized sensor data formats for robot integration (Section 3).

---

## Emerging Sources

### 11. Intel RealSense (2024) - D435i Depth Camera
**Citation**: Intel RealSense. (2024). *Intel RealSense D400 Series Datasheet*. [Online].

**URL**: [https://www.intelrealsense.com/depth-camera-d435i/](https://www.intelrealsense.com/depth-camera-d435i/)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: Technical specifications for D435i depth camera using structured light. Depth range 0.3-10m, ±2% accuracy @ 2m, 1280×720 depth resolution, 90 FPS, built-in BMI055 IMU (accelerometer + gyroscope).

**Relevance to Chapter**: Example of modern RGBD sensor with integrated IMU (Sections 1.1.2, 1.1.4).

---

### 12. Ouster (2024) - OS1 LiDAR Sensor
**Citation**: Ouster. (2024). *OS1 LiDAR Sensor Specifications*. [Online].

**URL**: [https://ouster.com/products/scanning-lidar/os1-sensor/](https://ouster.com/products/scanning-lidar/os1-sensor/)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: High-resolution 3D LiDAR with 64 or 128 channels, 120m range, 360° horizontal × 45° vertical FOV, 10-20 Hz scan rate, ±3cm accuracy, dual-return capability.

**Relevance to Chapter**: Example of high-end 3D LiDAR for robotics (Section 1.1.3).

---

### 13. Livox Technology (2024) - Mid-360 LiDAR
**Citation**: Livox Technology. (2024). *Mid-360 LiDAR: 360° FOV for Robotics*. [Online].

**URL**: [https://www.livoxtech.com/mid-360](https://www.livoxtech.com/mid-360)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: Low-cost ($500) solid-state LiDAR with 360° × 59° FOV, 40m range, non-repetitive scanning pattern (improves coverage compared to spinning LiDAR), 200,000 points/sec, ±2cm accuracy.

**Relevance to Chapter**: Affordable LiDAR option for humanoid robots (Section 1.1.3).

---

### 14. VectorNav (2024) - VN-100 IMU/AHRS
**Citation**: VectorNav. (2024). *VN-100 IMU/AHRS Technical Specifications*. [Online].

**URL**: [https://www.vectornav.com/products/detail/vn-100](https://www.vectornav.com/products/detail/vn-100)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: Industrial-grade 9-DOF IMU (3-axis accelerometer, gyroscope, magnetometer) with onboard Kalman filter. 0.05° RMS heading accuracy, 800 Hz output rate, temperature-calibrated sensors.

**Relevance to Chapter**: High-performance IMU for humanoid balance control (Section 1.1.4).

---

### 15. Boston Dynamics (2024) - Spot Robot Sensor Suite
**Citation**: Boston Dynamics. (2024). *Spot Robot Sensor Suite*. [Online].

**URL**: [https://www.bostondynamics.com/products/spot](https://www.bostondynamics.com/products/spot)

**Accessed**: December 2025

**Label**: [emerging]

**Summary**: Example of real-world multi-sensor integration in commercial robot. Spot uses 5 stereo camera pairs (360° vision), proprioceptive sensors (joint encoders, IMU), edge compute (NVIDIA Jetson), real-time SLAM.

**Relevance to Chapter**: Case study of sensor fusion architecture in deployed robot (Sections 1.3, 3).

---

## Citation Guidelines

**Established** sources are peer-reviewed publications or authoritative textbooks with stable citations. These form the theoretical foundation of the chapter.

**Emerging** sources are recent industry publications, datasheets, or current technical reports (2024-2025) representing cutting-edge hardware/software. These are clearly dated as technology evolves rapidly.

**Tool documentation** refers to official software library documentation used in code examples.

---

## Additional Resources for Further Reading

### Books
- Hartley, R., & Zisserman, A. (2004). *Multiple View Geometry in Computer Vision* (2nd ed.). Cambridge University Press. (Camera geometry, calibration, stereo vision)
- Szeliski, R. (2022). *Computer Vision: Algorithms and Applications* (2nd ed.). Springer. (Image processing, feature detection, 3D reconstruction)

### Survey Papers
- Cadena, C., et al. (2016). "Past, Present, and Future of Simultaneous Localization and Mapping: Toward the Robust-Perception Age." *IEEE Transactions on Robotics*, 32(6), 1309-1332. (SLAM survey covering sensors, algorithms, open problems)
- Wang, A., et al. (2023). "A Survey of Vision-Based Robotic Manipulation." *IEEE Transactions on Robotics*, 39(3), 1749-1768. (Vision sensors for manipulation tasks)

### Sensor Comparison Studies
- Pomerleau, F., et al. (2013). "Comparing ICP Variants on Real-World Data Sets." *Autonomous Robots*, 34(3), 133-148. (LiDAR registration algorithms)
- Sturm, J., et al. (2012). "A Benchmark for the Evaluation of RGB-D SLAM Systems." *Proc. IROS*, 573-580. (Depth camera evaluation dataset)

---

**Note**: All DOIs and URLs verified as of December 2025. For broken links, use DOI resolver (doi.org) or Wayback Machine (archive.org).
