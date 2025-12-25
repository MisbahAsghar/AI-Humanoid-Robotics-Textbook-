# ROS 2 Development

## Purpose
Guide creation of ROS 2 nodes, topics, services, actions, and launch files for distributed robot control systems with real-time communication.

## When to Use
- Building modular robot software architecture
- Creating publisher/subscriber nodes for sensor data
- Implementing service-based robot control APIs
- Setting up parameter servers and launch configurations
- Integrating multiple robot subsystems

## Book Chapters
- Chapter 3: ROS 2 Fundamentals

## Example Usage
```
User: "Create a ROS 2 node that publishes camera images and subscribes to motor commands"

Claude: Using the ros2-development skill, I'll reference Chapter 3's architecture:

1. Create package: `ros2 pkg create robot_vision --build-type ament_python`
2. Implement publisher node (Chapter 3, Section 1.2):
   - Use sensor_msgs/Image for camera data
   - Publish at 30 Hz to /camera/rgb/image topic
3. Implement subscriber node (Chapter 3, Section 1.3):
   - Subscribe to /motor/commands topic
   - Use std_msgs/Float64MultiArray for joint velocities

See Chapter 3, Sections 1.2-1.3 for complete code examples and QoS settings.
```
