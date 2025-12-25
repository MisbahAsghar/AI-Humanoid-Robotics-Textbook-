# Gazebo Simulation

## Purpose
Guide setup of Gazebo worlds, sensor plugins, physics configuration, and robot spawning for realistic physics-based simulation of humanoid robots.

## When to Use
- Creating custom simulation environments
- Adding sensor noise models (camera, LiDAR, IMU)
- Configuring physics engines and contact parameters
- Integrating Gazebo with ROS 2 control pipelines
- Testing robot behaviors before hardware deployment

## Book Chapters
- Chapter 5: Gazebo Simulation for Humanoids

## Example Usage
```
User: "Add realistic depth camera noise to my Gazebo simulation"

Claude: Using the gazebo-simulation skill, Chapter 5 Section 1.4.2 shows depth camera plugin configuration:

```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.3</near>
      <far>10.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </camera>
  <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so"/>
</sensor>
```

This adds ±1cm Gaussian noise. See Chapter 5, Section 1.4 for LiDAR and IMU noise models.
```
