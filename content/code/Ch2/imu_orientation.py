"""
IMU Data Interpretation and Orientation Estimation
Chapter 2: Humanoid Sensor Systems
Physical AI and Humanoid Robotics Textbook

Demonstrates reading and interpreting IMU data for orientation estimation.
"""

import numpy as np
import matplotlib.pyplot as plt

def simulate_imu_data(duration=10.0, sample_rate=100):
    """
    Simulate IMU data for a rotating robot.

    Args:
        duration: Simulation time (seconds)
        sample_rate: IMU sample rate (Hz)

    Returns:
        time: Time array (seconds)
        accel: Nx3 accelerometer data (m/s²)
        gyro: Nx3 gyroscope data (rad/s)
        true_angles: Nx3 ground truth orientation (roll, pitch, yaw in radians)
    """
    num_samples = int(duration * sample_rate)
    time = np.linspace(0, duration, num_samples)

    # Ground truth: robot rotates around Z-axis (yaw), slight pitch oscillation
    true_yaw = 0.5 * time  # 0.5 rad/s rotation rate
    true_pitch = 0.1 * np.sin(2 * np.pi * 0.5 * time)  # 0.5 Hz pitch oscillation
    true_roll = np.zeros(num_samples)

    true_angles = np.stack([true_roll, true_pitch, true_yaw], axis=1)

    # Gyroscope: measures angular velocity (derivative of angles)
    gyro_x = np.zeros(num_samples)  # No roll rate
    gyro_y = 0.1 * 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * time)  # Pitch rate
    gyro_z = 0.5 * np.ones(num_samples)  # Constant yaw rate

    # Add realistic gyro noise and bias
    gyro_noise = np.random.normal(0, 0.01, (num_samples, 3))  # 0.01 rad/s noise
    gyro_bias = np.array([0.005, -0.003, 0.002])  # Constant bias drift
    gyro = np.stack([gyro_x, gyro_y, gyro_z], axis=1) + gyro_noise + gyro_bias

    # Accelerometer: measures gravity + motion acceleration
    # In body frame: gravity rotates as robot rotates
    g = 9.81  # m/s²
    accel = np.zeros((num_samples, 3))

    for i in range(num_samples):
        # Gravity in world frame: [0, 0, -g]
        # Rotate into body frame using true angles
        pitch = true_pitch[i]
        roll = true_roll[i]

        # Simplified rotation (small angle approximation for clarity)
        accel[i, 0] = g * np.sin(pitch)  # Forward acceleration component
        accel[i, 1] = -g * np.sin(roll) * np.cos(pitch)  # Left component
        accel[i, 2] = -g * np.cos(roll) * np.cos(pitch)  # Up component

    # Add accelerometer noise
    accel += np.random.normal(0, 0.1, (num_samples, 3))  # 0.1 m/s² noise

    return time, accel, gyro, true_angles

def estimate_orientation_from_accelerometer(accel):
    """
    Estimate roll and pitch from accelerometer (gravity vector).
    Cannot estimate yaw (rotation around gravity axis).

    Args:
        accel: Nx3 accelerometer data (m/s²)

    Returns:
        roll: Roll angle (rotation around X-axis, radians)
        pitch: Pitch angle (rotation around Y-axis, radians)
    """
    # Normalize accelerometer vector (should point toward gravity)
    accel_norm = accel / np.linalg.norm(accel, axis=1, keepdims=True)

    # Extract roll and pitch from gravity vector direction
    roll = np.arctan2(accel_norm[:, 1], accel_norm[:, 2])
    pitch = np.arctan2(-accel_norm[:, 0], np.sqrt(accel_norm[:, 1]**2 + accel_norm[:, 2]**2))

    return roll, pitch

def integrate_gyroscope(gyro, dt):
    """
    Integrate gyroscope data to estimate orientation (suffers from drift).

    Args:
        gyro: Nx3 gyroscope data (rad/s)
        dt: Time step (seconds)

    Returns:
        angles: Nx3 integrated angles (roll, pitch, yaw in radians)
    """
    num_samples = gyro.shape[0]
    angles = np.zeros((num_samples, 3))

    for i in range(1, num_samples):
        # Simple Euler integration: angle[i] = angle[i-1] + gyro[i] * dt
        angles[i] = angles[i-1] + gyro[i] * dt

    return angles

def complementary_filter(accel, gyro, dt, alpha=0.98):
    """
    Fuse accelerometer and gyroscope using complementary filter.

    Args:
        accel: Nx3 accelerometer data (m/s²)
        gyro: Nx3 gyroscope data (rad/s)
        dt: Time step (seconds)
        alpha: Weight for gyro (0.98 typical: trust gyro 98%, accel 2%)

    Returns:
        fused_angles: Nx3 fused orientation (roll, pitch, yaw)
    """
    num_samples = accel.shape[0]
    fused_angles = np.zeros((num_samples, 3))

    for i in range(1, num_samples):
        # Get accelerometer-based angles (roll, pitch only)
        accel_roll, accel_pitch = estimate_orientation_from_accelerometer(accel[i:i+1])

        # Complementary filter for roll and pitch
        fused_angles[i, 0] = alpha * (fused_angles[i-1, 0] + gyro[i, 0] * dt) + (1 - alpha) * accel_roll
        fused_angles[i, 1] = alpha * (fused_angles[i-1, 1] + gyro[i, 1] * dt) + (1 - alpha) * accel_pitch

        # Yaw: only gyroscope available (accelerometer can't measure yaw)
        fused_angles[i, 2] = fused_angles[i-1, 2] + gyro[i, 2] * dt

    return fused_angles

def visualize_orientation_estimates(time, true_angles, gyro_angles, fused_angles):
    """
    Plot orientation estimates vs. ground truth.
    """
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    angle_names = ['Roll', 'Pitch', 'Yaw']

    for i, (ax, name) in enumerate(zip(axes, angle_names)):
        ax.plot(time, np.rad2deg(true_angles[:, i]), 'k-', linewidth=2, label='Ground Truth')
        ax.plot(time, np.rad2deg(gyro_angles[:, i]), 'r--', linewidth=1.5, alpha=0.7, label='Gyroscope Only (Drifts)')
        ax.plot(time, np.rad2deg(fused_angles[:, i]), 'b-', linewidth=1.5, label='Complementary Filter')

        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel(f'{name} (degrees)', fontsize=12)
        ax.set_title(f'{name} Angle Estimation', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('../../diagrams/Ch2/imu_orientation_estimation.png', dpi=150, bbox_inches='tight')
    plt.show()

def main():
    """
    Main function: simulate IMU data and estimate orientation.
    """
    print("=" * 60)
    print("IMU Orientation Estimation Demo")
    print("=" * 60)

    print("[1/4] Simulating IMU data (10 seconds @ 100 Hz)...")
    duration = 10.0
    sample_rate = 100  # Hz
    dt = 1.0 / sample_rate

    time, accel, gyro, true_angles = simulate_imu_data(duration, sample_rate)
    print(f"  ✓ Generated {len(time)} samples")
    print(f"  ✓ Accelerometer: {accel.shape[0]} samples of (ax, ay, az)")
    print(f"  ✓ Gyroscope: {gyro.shape[0]} samples of (ωx, ωy, ωz)")

    print("[2/4] Integrating gyroscope data (will drift)...")
    gyro_angles = integrate_gyroscope(gyro, dt)
    final_yaw_error = np.abs(gyro_angles[-1, 2] - true_angles[-1, 2])
    print(f"  ⚠ Yaw drift after 10s: {np.rad2deg(final_yaw_error):.2f} degrees")

    print("[3/4] Applying complementary filter (fusing accel + gyro)...")
    fused_angles = complementary_filter(accel, gyro, dt, alpha=0.98)
    final_pitch_error = np.abs(fused_angles[-1, 1] - true_angles[-1, 1])
    print(f"  ✓ Pitch error after 10s: {np.rad2deg(final_pitch_error):.2f} degrees")
    print("  → Complementary filter corrects gyro drift using accelerometer!")

    print("[4/4] Generating visualization...")
    visualize_orientation_estimates(time, true_angles, gyro_angles, fused_angles)
    print("  ✓ Saved: ../../diagrams/Ch2/imu_orientation_estimation.png")

    print("=" * 60)
    print("KEY INSIGHTS:")
    print("1. Gyroscope-only integration drifts due to bias")
    print("2. Accelerometer provides absolute pitch/roll (from gravity)")
    print("3. Complementary filter fuses both: short-term gyro, long-term accel")
    print("4. Yaw cannot be corrected by accelerometer (need magnetometer/vision)")
    print("=" * 60)

if __name__ == "__main__":
    main()
