"""
Point Cloud Visualization
Chapter 2: Humanoid Sensor Systems
Physical AI and Humanoid Robotics Textbook

Demonstrates creating, processing, and visualizing 3D point clouds
from depth sensors and LiDAR.
"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def generate_synthetic_point_cloud():
    """
    Generate synthetic point cloud simulating a room scan.

    Returns:
        points: Nx3 numpy array of (x, y, z) coordinates
        colors: Nx3 numpy array of (r, g, b) values [0-1]
    """
    points = []
    colors = []

    # Floor plane (z=0)
    x_floor = np.random.uniform(-3, 3, 500)
    y_floor = np.random.uniform(-3, 3, 500)
    z_floor = np.zeros(500)
    floor_points = np.stack([x_floor, y_floor, z_floor], axis=1)
    floor_colors = np.tile([0.6, 0.6, 0.6], (500, 1))  # Gray floor

    # Wall 1 (x=3)
    y_wall1 = np.random.uniform(-3, 3, 300)
    z_wall1 = np.random.uniform(0, 3, 300)
    x_wall1 = np.full(300, 3.0)
    wall1_points = np.stack([x_wall1, y_wall1, z_wall1], axis=1)
    wall1_colors = np.tile([0.8, 0.8, 0.7], (300, 1))  # Beige wall

    # Wall 2 (y=3)
    x_wall2 = np.random.uniform(-3, 3, 300)
    z_wall2 = np.random.uniform(0, 3, 300)
    y_wall2 = np.full(300, 3.0)
    wall2_points = np.stack([x_wall2, y_wall2, z_wall2], axis=1)
    wall2_colors = np.tile([0.7, 0.8, 0.8], (300, 1))  # Light blue wall

    # Object 1: Box (table)
    box_points = []
    for i in range(200):
        x = np.random.uniform(0.5, 1.5)
        y = np.random.uniform(0.5, 1.5)
        z = np.random.uniform(0.7, 0.8)  # Table height
        box_points.append([x, y, z])
    box_points = np.array(box_points)
    box_colors = np.tile([0.6, 0.3, 0.1], (200, 1))  # Brown table

    # Object 2: Sphere (ball on table)
    sphere_center = np.array([1.0, 1.0, 1.0])
    sphere_radius = 0.2
    sphere_points = []
    for i in range(150):
        # Random point on sphere surface
        theta = np.random.uniform(0, 2 * np.pi)
        phi = np.random.uniform(0, np.pi)
        x = sphere_radius * np.sin(phi) * np.cos(theta) + sphere_center[0]
        y = sphere_radius * np.sin(phi) * np.sin(theta) + sphere_center[1]
        z = sphere_radius * np.cos(phi) + sphere_center[2]
        sphere_points.append([x, y, z])
    sphere_points = np.array(sphere_points)
    sphere_colors = np.tile([1.0, 0.2, 0.2], (150, 1))  # Red ball

    # Combine all points
    all_points = np.vstack([floor_points, wall1_points, wall2_points, box_points, sphere_points])
    all_colors = np.vstack([floor_colors, wall1_colors, wall2_colors, box_colors, sphere_colors])

    # Add realistic sensor noise (±2cm Gaussian)
    noise = np.random.normal(0, 0.02, all_points.shape)
    all_points += noise

    return all_points, all_colors

def analyze_point_cloud(points):
    """
    Compute statistics for point cloud.

    Args:
        points: Nx3 numpy array

    Returns:
        stats: dict with centroid, bounding box, point count
    """
    centroid = np.mean(points, axis=0)
    min_bound = np.min(points, axis=0)
    max_bound = np.max(points, axis=0)

    return {
        'num_points': points.shape[0],
        'centroid': centroid,
        'min_bound': min_bound,
        'max_bound': max_bound,
        'dimensions': max_bound - min_bound
    }

def visualize_point_cloud_matplotlib(points, colors):
    """
    Create 2D projections of point cloud using matplotlib.
    """
    fig = plt.figure(figsize=(15, 5))

    # XY plane (top view)
    ax1 = fig.add_subplot(131)
    ax1.scatter(points[:, 0], points[:, 1], c=colors, s=1, alpha=0.6)
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_title('Top View (XY Plane)', fontsize=14, fontweight='bold')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # XZ plane (side view)
    ax2 = fig.add_subplot(132)
    ax2.scatter(points[:, 0], points[:, 2], c=colors, s=1, alpha=0.6)
    ax2.set_xlabel('X (m)', fontsize=12)
    ax2.set_ylabel('Z (m)', fontsize=12)
    ax2.set_title('Side View (XZ Plane)', fontsize=14, fontweight='bold')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    # 3D view
    ax3 = fig.add_subplot(133, projection='3d')
    ax3.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=1, alpha=0.6)
    ax3.set_xlabel('X (m)', fontsize=10)
    ax3.set_ylabel('Y (m)', fontsize=10)
    ax3.set_zlabel('Z (m)', fontsize=10)
    ax3.set_title('3D View', fontsize=14, fontweight='bold')

    plt.tight_layout()
    plt.savefig('../../diagrams/Ch2/point_cloud_projections.png', dpi=150, bbox_inches='tight')
    plt.show()

def visualize_point_cloud_open3d(points, colors):
    """
    Interactive 3D visualization using Open3D.
    """
    # Create Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Add coordinate frame (X=red, Y=green, Z=blue)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    # Visualize (opens interactive window)
    print("  → Opening Open3D interactive viewer (close window to continue)...")
    o3d.visualization.draw_geometries([pcd, coordinate_frame],
                                      window_name="Point Cloud Visualization",
                                      width=1024, height=768,
                                      left=50, top=50)

def main():
    """
    Main function: generate, analyze, and visualize point cloud.
    """
    print("=" * 60)
    print("Point Cloud Visualization Demo")
    print("=" * 60)

    print("[1/4] Generating synthetic point cloud (simulated room scan)...")
    points, colors = generate_synthetic_point_cloud()
    print(f"  ✓ Generated {points.shape[0]} points")

    print("[2/4] Analyzing point cloud statistics...")
    stats = analyze_point_cloud(points)
    print(f"  ✓ Centroid: ({stats['centroid'][0]:.2f}, {stats['centroid'][1]:.2f}, {stats['centroid'][2]:.2f}) m")
    print(f"  ✓ Bounding box: {stats['dimensions'][0]:.2f} x {stats['dimensions'][1]:.2f} x {stats['dimensions'][2]:.2f} m")
    print(f"  ✓ Point count: {stats['num_points']}")

    print("[3/4] Creating matplotlib projections...")
    visualize_point_cloud_matplotlib(points, colors)
    print("  ✓ Saved: ../../diagrams/Ch2/point_cloud_projections.png")

    print("[4/4] Launching Open3D interactive viewer...")
    try:
        visualize_point_cloud_open3d(points, colors)
    except Exception as e:
        print(f"  ⚠ Open3D visualization failed: {e}")
        print("  → Skipping interactive viewer (matplotlib visualization saved)")

    print("=" * 60)
    print("✓ Point cloud visualization demo complete!")
    print("=" * 60)

if __name__ == "__main__":
    main()
