# Chapter 2 Code Examples

**Chapter**: Humanoid Sensor Systems
**Purpose**: Hands-on demonstrations of sensor data processing and interpretation

---

## Quick Start

### 1. Install Dependencies

```bash
pip install numpy matplotlib opencv-python open3d scipy
```

### 2. Run Examples

```bash
# Camera data processing
python camera_capture.py

# Point cloud visualization
python point_cloud_visualization.py

# IMU orientation estimation
python imu_orientation.py
```

---

## Script Descriptions

### `camera_capture.py`
**Demonstrates**: RGB camera data capture, color space conversion, image processing

**What it does**:
- Captures frame from webcam (or generates synthetic test pattern)
- Converts between color spaces (BGR ↔ RGB ↔ Grayscale)
- Applies image processing: edge detection (Canny), Gaussian blur
- Generates 4-panel comparison figure

**Output**: `../../diagrams/Ch2/camera_processing.png`

**Key Insights**:
- OpenCV uses BGR by default; matplotlib uses RGB
- Canny edge detection finds intensity gradients
- Gaussian blur reduces noise before processing
- Fallback to synthetic data if webcam unavailable

---

### `point_cloud_visualization.py`
**Demonstrates**: 3D point cloud generation, processing, and visualization

**What it does**:
- Generates synthetic 3D point cloud (simulated room: walls, floor, table, sphere)
- Computes point cloud statistics (centroid, bounding box, dimensions)
- Creates matplotlib 2D projections (top view XY, side view XZ, 3D view)
- Launches interactive Open3D viewer with coordinate frame
- Adds realistic ±2cm Gaussian sensor noise

**Outputs**:
- `../../diagrams/Ch2/point_cloud_projections.png` (3-panel matplotlib figure)
- Open3D interactive window (close to continue)

**Key Insights**:
- Point clouds are Nx3 arrays of (x, y, z) coordinates
- Coordinate frames: X=red, Y=green, Z=blue
- Sensor noise degrades geometric precision
- Multiple visualization methods (2D projections + 3D interactive)

---

### `imu_orientation.py`
**Demonstrates**: IMU data interpretation and orientation estimation using sensor fusion

**What it does**:
- Simulates IMU data for rotating robot (accelerometer + gyroscope)
- Integrates gyroscope → shows drift accumulation over time
- Estimates orientation from accelerometer gravity vector
- Applies complementary filter (98% gyro, 2% accel) to correct drift
- Compares: ground truth vs. gyro-only vs. fused estimates

**Output**: `../../diagrams/Ch2/imu_orientation_estimation.png` (3-panel time series: roll, pitch, yaw)

**Key Insights**:
- Gyroscope integration drifts due to bias (unbounded error growth)
- Accelerometer provides absolute pitch/roll (measures gravity direction)
- Complementary filter fuses sensors: short-term gyro, long-term accel
- Yaw cannot be measured by accelerometer (need magnetometer or vision)
- Sensor fusion is essential for long-term orientation tracking

---

## Expected Outputs

All scripts generate publication-quality figures saved to `../../diagrams/Ch2/`:

1. **camera_processing.png**: 4-panel (original RGB, grayscale, edge detection, blur)
2. **point_cloud_projections.png**: 3-panel (top view XY, side view XZ, 3D view)
3. **imu_orientation_estimation.png**: 3-panel time series (roll, pitch, yaw angles)

---

## Troubleshooting

### Common Errors

**1. ModuleNotFoundError: No module named 'cv2'**
- **Cause**: OpenCV not installed or installed incorrectly
- **Fix**: `pip install opencv-python` (not `pip install cv2`)

**2. ImportError: DLL load failed (Windows)**
- **Cause**: Missing Visual C++ Redistributable
- **Fix**: Install VC++ Redist: https://aka.ms/vs/17/release/vc_redist.x64.exe

**3. "Cannot open webcam" (camera_capture.py)**
- **Cause**: Camera in use by another app (Zoom, Teams) or not connected
- **Fix**: Close other camera apps, or script will use synthetic data fallback

**4. Open3D window doesn't appear (point_cloud_visualization.py)**
- **Cause**: Missing Qt5 libraries or GPU driver issue
- **Fix Linux**: `sudo apt install python3-pyqt5` or `pip install pyqt5`
- **Fix GPU**: Set `export OPEN3D_CPU_RENDERING=1` before running
- **Alternative**: Script saves matplotlib figure even if Open3D fails

**5. Matplotlib backend errors (headless systems / SSH)**
- **Cause**: No display server available
- **Fix**: Set non-interactive backend before importing pyplot:
  ```python
  import matplotlib
  matplotlib.use('Agg')  # Non-interactive backend
  import matplotlib.pyplot as plt
  ```

**6. "Segmentation fault" on Linux (Open3D)**
- **Cause**: GPU driver incompatibility
- **Fix**: Use CPU rendering: `export OPEN3D_CPU_RENDERING=1`

---

## Modifying Examples

### Exercise 1: Real Webcam Integration
Extend `camera_capture.py` to capture multiple frames and track object motion:

```python
def capture_video_stream(duration=5.0, fps=30):
    """Capture video stream for specified duration."""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, fps)
    num_frames = int(duration * fps)
    frames = []

    for i in range(num_frames):
        ret, frame = cap.read()
        if ret:
            frames.append(frame)

    cap.release()
    return frames
```

### Exercise 2: Point Cloud Downsampling
Extend `point_cloud_visualization.py` to implement voxel downsampling:

```python
def downsample_point_cloud(pcd, voxel_size=0.05):
    """Reduce point cloud density using voxel grid filter."""
    pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"Downsampled: {len(pcd.points)} → {len(pcd_downsampled.points)} points")
    return pcd_downsampled
```

### Exercise 3: Kalman Filter for IMU
Extend `imu_orientation.py` to implement Extended Kalman Filter (EKF):

```python
from filterpy.kalman import ExtendedKalmanFilter

def ekf_orientation_estimation(accel, gyro, dt):
    """Estimate orientation using Extended Kalman Filter."""
    # State: [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
    ekf = ExtendedKalmanFilter(dim_x=6, dim_z=3)
    # Define motion model, measurement model, noise covariances
    # ... (implementation left as exercise)
    pass
```

---

## System Requirements

**Minimum**:
- Python 3.8+
- 4GB RAM
- 200MB disk space

**Recommended**:
- Python 3.10+
- 8GB RAM (for large point clouds)
- Ubuntu 22.04 or Windows 10+
- USB webcam (optional, scripts have synthetic data fallback)

**Dependencies**:
- numpy >= 1.24
- matplotlib >= 3.7
- opencv-python >= 4.8
- open3d >= 0.17
- scipy >= 1.10

---

## Related Chapter Content

These code examples support concepts in:
- Section 1.1: Sensor Types (RGB cameras, depth cameras, LiDAR, IMU)
- Section 1.2: Sensor Characteristics (FOV, resolution, noise, calibration)
- Section 1.3: Sensor Fusion Basics (complementary filter, Kalman filter)
- Section 2.2: Reading and Processing Camera Data
- Section 2.3: Visualizing 3D Point Clouds
- Section 2.4: IMU Data Interpretation and Orientation Estimation

For theoretical background, see `content/chapters/Ch2.md` sections 1.1-2.4.

---

## Contact / Issues

If code examples don't run as expected:
1. Verify Python version: `python --version` (should be 3.8+)
2. Verify dependencies: `pip list | grep -E "numpy|matplotlib|opencv|open3d"`
3. Check diagram output directory exists: `mkdir -p ../../diagrams/Ch2/`
4. Review troubleshooting section above
5. Check for error messages mentioning missing libraries or hardware

---

**Last Updated**: 2025-12-23
**Tested On**: Ubuntu 22.04, Python 3.10, numpy 1.24, matplotlib 3.7, opencv-python 4.8, open3d 0.17
