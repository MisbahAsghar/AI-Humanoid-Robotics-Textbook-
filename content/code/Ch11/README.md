# Chapter 11 Code Examples

**Chapter**: Navigation and Manipulation
**Purpose**: Nav2 autonomous navigation + MoveIt pick-and-place integration

---

## Requirements

- ROS 2 Humble
- Nav2: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- MoveIt: `sudo apt install ros-humble-moveit`
- Gazebo/Isaac Sim (for testing)

---

## Examples

### `nav2_params.yaml`
Nav2 configuration: controller frequency, planner plugins, costmap parameters

### `pick_and_place.py` (~80 lines)
MoveIt-based pick-and-place with approach/grasp/lift/place primitives

---

## Running

```bash
# Launch Nav2
ros2 launch nav2_bringup navigation_launch.py params_file:=nav2_params.yaml

# Run pick-and-place
ros2 run manipulation_demos pick_and_place
```

---

**Last Updated**: 2025-12-23
