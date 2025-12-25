# URDF Modeling

## Purpose
Guide creation of accurate URDF robot descriptions including links, joints, collision geometry, inertia tensors, and visual meshes for simulation and kinematics.

## When to Use
- Designing a new robot model from scratch
- Converting CAD models to URDF format
- Debugging joint configurations or collision issues
- Calculating inertia matrices for robot links
- Spawning robots in Gazebo or RViz

## Book Chapters
- Chapter 4: URDF and Robot Description

## Example Usage
```
User: "Help me calculate the inertia tensor for a cylindrical robot link"

Claude: Using the urdf-modeling skill, Chapter 4 Section 1.2.2 provides formulas:

For a cylinder (mass m, radius r, height h) along Z-axis:
- I_xx = I_yy = m(3r² + h²)/12
- I_zz = mr²/2
- Off-diagonal terms = 0

Example: m=2kg, r=0.05m, h=0.3m
- I_xx = I_yy = 0.01625 kg·m²
- I_zz = 0.0025 kg·m²

URDF format:
```xml
<inertial>
  <mass value="2.0"/>
  <inertia ixx="0.01625" ixy="0.0" ixz="0.0"
           iyy="0.01625" iyz="0.0" izz="0.0025"/>
</inertial>
```

See Chapter 4, Section 1.2 for box and sphere formulas.
```
