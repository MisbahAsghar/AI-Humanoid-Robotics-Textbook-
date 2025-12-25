"""
LiDAR Sensor Visualization
Chapter 1: Introduction to Physical AI
Physical AI and Humanoid Robotics Textbook

This script simulates and visualizes 2D LiDAR sensor data, demonstrating how
Physical AI systems perceive environments using range sensors.
"""

import numpy as np
import matplotlib.pyplot as plt

def simulate_2d_lidar_scan(num_points=360, max_range=10.0, obstacles=None):
    """
    Simulate a 2D LiDAR scan in a simple environment.

    Args:
        num_points: Number of laser beams (angular resolution)
        max_range: Maximum sensing range (meters)
        obstacles: List of (x, y, radius) tuples for circular obstacles

    Returns:
        angles: Beam angles (radians)
        ranges: Measured distances (meters)
    """
    angles = np.linspace(0, 2*np.pi, num_points, endpoint=False)
    ranges = np.full(num_points, max_range)  # Default: no obstacle

    if obstacles is None:
        # Create a simple room with walls
        obstacles = [
            (0, 5, 0.1),    # North wall (as a far point)
            (0, -5, 0.1),   # South wall
            (5, 0, 0.1),    # East wall
            (-5, 0, 0.1),   # West wall
            (2, 2, 0.5),    # Obstacle 1
            (-2, 1, 0.7),   # Obstacle 2
        ]

    # Ray-casting to find intersections
    for i, angle in enumerate(angles):
        # Ray direction
        dx, dy = np.cos(angle), np.sin(angle)

        # Check intersection with each obstacle
        min_dist = max_range
        for ox, oy, radius in obstacles:
            # Ray-circle intersection (simplified)
            # Distance from origin to obstacle center
            dist_to_center = np.sqrt(ox**2 + oy**2)
            # Project obstacle onto ray direction
            projection = ox*dx + oy*dy
            if projection > 0:  # Obstacle is in ray direction
                # Perpendicular distance from ray to obstacle center
                if projection < dist_to_center:
                    perp_dist = np.sqrt(dist_to_center**2 - projection**2)
                else:
                    perp_dist = dist_to_center
                if perp_dist < radius:  # Ray intersects obstacle
                    intersection_dist = projection - np.sqrt(max(0, radius**2 - perp_dist**2))
                    if intersection_dist < min_dist and intersection_dist > 0:
                        min_dist = intersection_dist

        ranges[i] = min_dist

    # Add realistic noise
    ranges += np.random.normal(0, 0.02, ranges.shape)  # ±2cm noise
    ranges = np.clip(ranges, 0.1, max_range)

    return angles, ranges

def visualize_lidar_scan():
    """
    Visualize a 2D LiDAR scan in polar and Cartesian coordinates.
    """
    angles, ranges = simulate_2d_lidar_scan()

    # Convert polar to Cartesian
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    # Create visualization
    fig = plt.figure(figsize=(14, 6))

    # Polar plot
    ax1 = fig.add_subplot(121, projection='polar')
    ax1.plot(angles, ranges, 'b.', markersize=2)
    ax1.set_ylim(0, 10)
    ax1.set_title('LiDAR Scan (Polar Coordinates)', fontsize=14, fontweight='bold')
    ax1.set_theta_zero_location('N')

    # Cartesian plot (bird's eye view)
    ax2 = fig.add_subplot(122)
    ax2.plot(x, y, 'r.', markersize=2, label='LiDAR Points')
    ax2.plot(0, 0, 'go', markersize=10, label='Robot Position', zorder=5)
    ax2.set_xlim(-6, 6)
    ax2.set_ylim(-6, 6)
    ax2.set_xlabel('X (meters)', fontsize=12)
    ax2.set_ylabel('Y (meters)', fontsize=12)
    ax2.set_title('LiDAR Scan (Cartesian / Bird\'s Eye View)', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')

    plt.tight_layout()
    plt.savefig('../../diagrams/Ch1/lidar_scan_visualization.png', dpi=150, bbox_inches='tight')
    plt.show()

    print("✓ LiDAR visualization complete")
    print(f"  Scan statistics: {len(ranges)} points, range = 0.1-10m")
    print(f"  Noise model: Gaussian ±2cm (realistic for indoor LiDAR)")
    print("  Saved: ../../diagrams/Ch1/lidar_scan_visualization.png")

if __name__ == "__main__":
    print("="*60)
    print("LiDAR Sensor Visualization")
    print("Chapter 1: Introduction to Physical AI")
    print("="*60 + "\n")

    visualize_lidar_scan()
