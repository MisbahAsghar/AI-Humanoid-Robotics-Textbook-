"""
Camera Capture and Processing
Chapter 2: Humanoid Sensor Systems
Physical AI and Humanoid Robotics Textbook

Demonstrates reading RGB camera data, color space conversion,
and basic image processing.
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt

def capture_webcam_frame():
    """Capture a single frame from default webcam."""
    # Open video capture (0 = default camera)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError("Cannot open webcam. Check camera connection.")

    # Set camera properties (optional)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Capture frame
    ret, frame = cap.read()
    cap.release()

    if not ret:
        raise RuntimeError("Failed to capture frame from webcam.")

    return frame

def process_camera_frame(frame):
    """
    Apply image processing pipeline to camera frame.

    Args:
        frame: BGR image from cv2.VideoCapture (HxWx3 numpy array)

    Returns:
        processed_results: dict with processed images
    """
    # OpenCV uses BGR by default; convert to RGB for matplotlib
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Convert to grayscale (luminance)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Edge detection (Canny algorithm)
    edges = cv2.Canny(gray, threshold1=50, threshold2=150)

    # Gaussian blur (noise reduction)
    blurred = cv2.GaussianBlur(gray, ksize=(5, 5), sigmaX=1.5)

    return {
        'rgb': frame_rgb,
        'gray': gray,
        'edges': edges,
        'blurred': blurred
    }

def simulate_camera_data():
    """
    Create synthetic camera data if no webcam available.
    """
    # Generate checkerboard test pattern
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    square_size = 40

    for i in range(0, 480, square_size):
        for j in range(0, 640, square_size):
            if ((i // square_size) + (j // square_size)) % 2 == 0:
                img[i:i+square_size, j:j+square_size] = [255, 255, 255]

    # Add colored circle
    cv2.circle(img, (320, 240), 80, (255, 0, 0), -1)  # Blue circle (BGR)

    return img

def visualize_camera_processing():
    """
    Main function: capture and visualize camera processing pipeline.
    """
    print("=" * 60)
    print("Camera Data Processing Demo")
    print("=" * 60)

    # Try to capture from webcam; fallback to synthetic data
    try:
        print("[1/3] Capturing frame from webcam...")
        frame_bgr = capture_webcam_frame()
        print("  ✓ Webcam frame captured (640x480)")
    except RuntimeError as e:
        print(f"  ⚠ Webcam capture failed: {e}")
        print("  → Using synthetic test pattern instead")
        frame_bgr = simulate_camera_data()

    # Process frame
    print("[2/3] Processing image...")
    results = process_camera_frame(frame_bgr)
    print("  ✓ Applied RGB conversion, grayscale, edge detection, blur")

    # Visualize
    print("[3/3] Generating visualization...")
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    axes[0, 0].imshow(results['rgb'])
    axes[0, 0].set_title('Original (RGB)', fontsize=14, fontweight='bold')
    axes[0, 0].axis('off')

    axes[0, 1].imshow(results['gray'], cmap='gray')
    axes[0, 1].set_title('Grayscale', fontsize=14, fontweight='bold')
    axes[0, 1].axis('off')

    axes[1, 0].imshow(results['edges'], cmap='gray')
    axes[1, 0].set_title('Edge Detection (Canny)', fontsize=14, fontweight='bold')
    axes[1, 0].axis('off')

    axes[1, 1].imshow(results['blurred'], cmap='gray')
    axes[1, 1].set_title('Gaussian Blur (σ=1.5)', fontsize=14, fontweight='bold')
    axes[1, 1].axis('off')

    plt.tight_layout()
    plt.savefig('../../diagrams/Ch2/camera_processing.png', dpi=150, bbox_inches='tight')
    plt.show()

    print("=" * 60)
    print("✓ Camera processing demo complete!")
    print("  Saved: ../../diagrams/Ch2/camera_processing.png")
    print("=" * 60)

if __name__ == "__main__":
    visualize_camera_processing()
