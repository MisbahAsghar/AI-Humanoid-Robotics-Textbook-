"""
Physical Constraint Analysis
Chapter 1: Introduction to Physical AI
Physical AI and Humanoid Robotics Textbook

This script demonstrates quantitative analysis of physical constraints
in robot systems: latency impacts, power consumption, and sensor noise.
"""

import numpy as np
import matplotlib.pyplot as plt

def calculate_latency_impact(robot_speed, control_latency_ms):
    """
    Calculate how far a robot moves during control latency.

    Args:
        robot_speed: Robot speed (m/s)
        control_latency_ms: Control loop latency (milliseconds)

    Returns:
        distance_traveled: Distance traveled during latency (meters)
    """
    latency_seconds = control_latency_ms / 1000.0
    distance_traveled = robot_speed * latency_seconds
    return distance_traveled

def demonstrate_latency_impact():
    """
    Visualize how control latency affects robot reaction distance.
    """
    robot_speeds = np.linspace(0.1, 3.0, 30)  # 0.1 to 3 m/s (human walking to running)
    latencies = [10, 50, 100, 200]  # milliseconds

    plt.figure(figsize=(10, 6))

    for latency in latencies:
        distances = [calculate_latency_impact(speed, latency) for speed in robot_speeds]
        plt.plot(robot_speeds, np.array(distances)*100, linewidth=2, label=f'{latency} ms latency')

    plt.xlabel('Robot Speed (m/s)', fontsize=12)
    plt.ylabel('Reaction Distance (cm)', fontsize=12)
    plt.title('Impact of Control Latency on Reaction Distance', fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.axhline(y=10, color='r', linestyle='--', alpha=0.5)
    plt.text(2.5, 12, '10 cm (acceptable)', fontsize=9, color='r')
    plt.axhline(y=30, color='orange', linestyle='--', alpha=0.5)
    plt.text(2.5, 32, '30 cm (risky)', fontsize=9, color='orange')

    plt.savefig('../../diagrams/Ch1/latency_impact.png', dpi=150, bbox_inches='tight')
    plt.show()

    # Print example calculations
    print("✓ Latency impact analysis complete")
    print("\nExample: Humanoid walking at 1 m/s")
    for latency in latencies:
        dist = calculate_latency_impact(1.0, latency) * 100
        print(f"  {latency} ms latency → {dist:.1f} cm travel before reaction")

    print("\n⚠ Key Insight: At 100 ms latency, a humanoid moving at 1 m/s")
    print("   travels 10 cm before reacting. This can prevent balance recovery!")
    print("  Saved: ../../diagrams/Ch1/latency_impact.png")

def calculate_power_consumption(compute_power_watts, runtime_hours, battery_capacity_wh):
    """
    Calculate robot runtime given power consumption.

    Args:
        compute_power_watts: Power consumption of compute (W)
        runtime_hours: Desired runtime (hours)
        battery_capacity_wh: Battery capacity (watt-hours)

    Returns:
        feasibility: dict with analysis
    """
    total_energy_needed = compute_power_watts * runtime_hours
    feasibility = {
        'energy_needed_wh': total_energy_needed,
        'battery_capacity_wh': battery_capacity_wh,
        'feasible': total_energy_needed <= battery_capacity_wh,
        'actual_runtime_hours': battery_capacity_wh / compute_power_watts if compute_power_watts > 0 else float('inf')
    }
    return feasibility

def demonstrate_power_constraints():
    """
    Analyze power consumption trade-offs for different AI models.
    """
    # Battery capacity for typical humanoid robot
    battery_capacity = 500  # watt-hours (e.g., Tesla Bot uses ~2.3 kWh, but 500Wh for smaller robots)

    # AI model power consumption examples
    models = {
        'TinyML (MobileNet)': 2,   # Watts
        'Edge GPU (Jetson Nano)': 10,
        'Medium Model (Jetson Xavier)': 30,
        'Large Model (Desktop GPU)': 150,
        'Vision Transformer (Full)': 300,
    }

    print("\n" + "="*60)
    print("POWER CONSUMPTION ANALYSIS FOR HUMANOID ROBOT")
    print("="*60)
    print(f"Battery Capacity: {battery_capacity} Wh\n")

    runtimes = []
    model_names = []

    for model_name, power in models.items():
        analysis = calculate_power_consumption(power, 1.0, battery_capacity)
        runtime = analysis['actual_runtime_hours']
        runtimes.append(runtime)
        model_names.append(model_name)

        print(f"{model_name}:")
        print(f"  Power: {power} W")
        print(f"  Runtime: {runtime:.2f} hours ({runtime*60:.0f} minutes)")
        if runtime < 1.0:
            print(f"  ⚠ WARNING: Less than 1 hour runtime!")
        print()

    # Visualize
    plt.figure(figsize=(10, 6))
    colors = ['green' if r >= 2 else 'orange' if r >= 1 else 'red' for r in runtimes]
    plt.barh(model_names, runtimes, color=colors)
    plt.xlabel('Runtime (hours)', fontsize=12)
    plt.title('Robot Runtime vs. AI Model Power Consumption', fontsize=14, fontweight='bold')
    plt.axvline(x=2, color='g', linestyle='--', alpha=0.5)
    plt.text(2.2, 4, '2 hours\n(acceptable)', fontsize=9, color='g')
    plt.axvline(x=1, color='orange', linestyle='--', alpha=0.5)
    plt.text(1.1, 2, '1 hour\n(minimum)', fontsize=9, color='orange')
    plt.grid(True, alpha=0.3, axis='x')
    plt.tight_layout()
    plt.savefig('../../diagrams/Ch1/power_consumption_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()

    print("="*60)
    print("KEY INSIGHT: Model selection is a power-runtime trade-off!")
    print("Large vision models may be too power-hungry for mobile robots.")
    print("="*60)
    print("  Saved: ../../diagrams/Ch1/power_consumption_analysis.png")

def simulate_sensor_noise_impact():
    """
    Demonstrate how sensor noise affects measurement accuracy.
    """
    # True distance to obstacle
    true_distance = 5.0  # meters

    # Simulate 1000 LiDAR measurements with noise
    num_measurements = 1000
    noise_std = 0.02  # ±2cm standard deviation (typical for indoor LiDAR)
    measurements = np.random.normal(true_distance, noise_std, num_measurements)

    # Calculate statistics
    mean_measured = np.mean(measurements)
    std_measured = np.std(measurements)
    error = mean_measured - true_distance

    # Visualize distribution
    plt.figure(figsize=(10, 6))
    plt.hist(measurements, bins=50, density=True, alpha=0.7, edgecolor='black')
    plt.axvline(true_distance, color='r', linestyle='--', linewidth=2, label=f'True Distance: {true_distance} m')
    plt.axvline(mean_measured, color='g', linestyle='--', linewidth=2, label=f'Mean Measured: {mean_measured:.4f} m')
    plt.xlabel('Measured Distance (m)', fontsize=12)
    plt.ylabel('Probability Density', fontsize=12)
    plt.title('LiDAR Measurement Distribution (Gaussian Noise Model)', fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('../../diagrams/Ch1/sensor_noise_distribution.png', dpi=150, bbox_inches='tight')
    plt.show()

    print("\n" + "="*60)
    print("SENSOR NOISE IMPACT ANALYSIS")
    print("="*60)
    print(f"True Distance: {true_distance} m")
    print(f"Mean Measured: {mean_measured:.4f} m (error: {error*100:.2f} cm)")
    print(f"Std Deviation: {std_measured*100:.2f} cm")
    print(f"95% Confidence: ±{1.96*std_measured*100:.2f} cm")
    print("\n⚠ Implications:")
    print("  - Single measurement can be off by several cm")
    print("  - Average multiple measurements to reduce noise")
    print("  - Kalman filtering helps estimate true state")
    print("="*60)
    print("  Saved: ../../diagrams/Ch1/sensor_noise_distribution.png")

if __name__ == "__main__":
    print("="*60)
    print("Physical AI Constraint Demonstrations")
    print("Chapter 1: Introduction to Physical AI")
    print("="*60 + "\n")

    print("[1/3] Latency Impact Analysis...")
    demonstrate_latency_impact()

    print("\n[2/3] Power Consumption Analysis...")
    demonstrate_power_constraints()

    print("\n[3/3] Sensor Noise Impact Analysis...")
    simulate_sensor_noise_impact()

    print("\n✓ All constraint demonstrations complete!")
    print("  Review generated figures in ../../diagrams/Ch1/")
