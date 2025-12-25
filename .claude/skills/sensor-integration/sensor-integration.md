# Sensor Integration

## Purpose
Guide implementation of RGB cameras, depth cameras, LiDAR, and IMU sensors for humanoid robots, including calibration, sensor fusion, and coordinate frame transformations.

## When to Use
- Selecting sensors for a robot platform
- Implementing sensor drivers and processing pipelines
- Calibrating cameras or multi-sensor systems
- Understanding sensor noise models and error propagation
- Performing coordinate transformations between sensor frames

## Book Chapters
- Chapter 2: Sensors for Humanoid Robotics

## Example Usage
```
User: "How do I fuse IMU and camera data for better orientation estimates?"

Claude: Using the sensor-integration skill, Chapter 2 Section 1.3.1 covers sensor fusion:

Complementary Filter approach:
- Gyroscope: accurate short-term, drifts long-term
- Accelerometer: accurate long-term, noisy short-term
- Formula: θ_fused(t) = α·(θ_gyro + ω·Δt) + (1-α)·θ_accel

Typical α = 0.98 (trust gyro 98%, accel 2%)

For production systems, Chapter 2 also covers Kalman filtering for adaptive fusion.

See Chapter 2, Section 1.3 for implementation details.
```
