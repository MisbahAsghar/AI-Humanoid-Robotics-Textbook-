"""
Camera Sensor Noise Visualization
Chapter 1: Introduction to Physical AI
Physical AI and Humanoid Robotics Textbook

This script demonstrates how different types of camera noise affect image quality
and computer vision tasks in Physical AI systems.
"""

import numpy as np
import matplotlib.pyplot as plt
import cv2

def simulate_camera_noise(image, noise_type='gaussian', noise_level=25):
    """
    Simulate realistic camera noise on an image.

    Args:
        image: Input image (numpy array)
        noise_type: 'gaussian', 'salt_pepper', or 'motion_blur'
        noise_level: Noise intensity (0-100)

    Returns:
        Noisy image
    """
    if noise_type == 'gaussian':
        # Simulate shot noise and read noise
        noise = np.random.normal(0, noise_level, image.shape)
        noisy_image = image + noise
        return np.clip(noisy_image, 0, 255).astype(np.uint8)

    elif noise_type == 'salt_pepper':
        # Simulate pixel defects
        noisy_image = image.copy()
        prob = noise_level / 1000.0
        # Salt (white pixels)
        salt = np.random.rand(*image.shape[:2]) < prob
        noisy_image[salt] = 255
        # Pepper (black pixels)
        pepper = np.random.rand(*image.shape[:2]) < prob
        noisy_image[pepper] = 0
        return noisy_image

    elif noise_type == 'motion_blur':
        # Simulate camera/object motion during exposure
        kernel_size = max(3, int(noise_level / 5))
        kernel = np.zeros((kernel_size, kernel_size))
        kernel[int((kernel_size-1)/2), :] = np.ones(kernel_size)
        kernel = kernel / kernel_size
        return cv2.filter2D(image, -1, kernel)

    return image

def visualize_sensor_noise():
    """
    Demonstrate the impact of sensor noise on perception.
    """
    # Create a synthetic scene (checkerboard pattern)
    checkerboard = np.zeros((400, 400, 3), dtype=np.uint8)
    square_size = 50
    for i in range(0, 400, square_size):
        for j in range(0, 400, square_size):
            if (i // square_size + j // square_size) % 2 == 0:
                checkerboard[i:i+square_size, j:j+square_size] = 200

    # Add a simple object (circle)
    cv2.circle(checkerboard, (200, 200), 60, (100, 150, 200), -1)

    # Simulate different noise types
    gaussian_noisy = simulate_camera_noise(checkerboard, 'gaussian', 25)
    salt_pepper_noisy = simulate_camera_noise(checkerboard, 'salt_pepper', 10)
    motion_blur_noisy = simulate_camera_noise(checkerboard, 'motion_blur', 20)

    # Visualize
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    axes[0, 0].imshow(cv2.cvtColor(checkerboard, cv2.COLOR_BGR2RGB))
    axes[0, 0].set_title('Clean Image (Ideal)', fontsize=14, fontweight='bold')
    axes[0, 0].axis('off')

    axes[0, 1].imshow(cv2.cvtColor(gaussian_noisy, cv2.COLOR_BGR2RGB))
    axes[0, 1].set_title('Gaussian Noise (Low Light)', fontsize=14, fontweight='bold')
    axes[0, 1].axis('off')

    axes[1, 0].imshow(cv2.cvtColor(salt_pepper_noisy, cv2.COLOR_BGR2RGB))
    axes[1, 0].set_title('Salt & Pepper Noise (Sensor Defects)', fontsize=14, fontweight='bold')
    axes[1, 0].axis('off')

    axes[1, 1].imshow(cv2.cvtColor(motion_blur_noisy, cv2.COLOR_BGR2RGB))
    axes[1, 1].set_title('Motion Blur (Fast Movement)', fontsize=14, fontweight='bold')
    axes[1, 1].axis('off')

    plt.tight_layout()
    plt.savefig('../../diagrams/Ch1/camera_noise_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()

    print("✓ Camera noise visualization complete")
    print("  Observe: Noise makes object detection and edge detection much harder")
    print("  Saved: ../../diagrams/Ch1/camera_noise_comparison.png")

if __name__ == "__main__":
    print("="*60)
    print("Camera Sensor Noise Visualization")
    print("Chapter 1: Introduction to Physical AI")
    print("="*60 + "\n")

    visualize_sensor_noise()
