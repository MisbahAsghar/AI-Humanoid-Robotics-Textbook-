# Chapter 1 Code Examples

**Chapter**: Introduction to Physical AI
**Purpose**: Hands-on demonstrations of Physical AI concepts

---

## Quick Start

### 1. Install Dependencies

```bash
pip install numpy matplotlib opencv-python scipy
```

### 2. Run Examples

```bash
# Camera noise visualization
python camera_visualization.py

# LiDAR sensor visualization
python lidar_visualization.py

# Physical constraints analysis (all 3 demos)
python physical_constraints.py
```

---

## Script Descriptions

### `camera_visualization.py`
**Demonstrates**: How camera sensor noise affects computer vision

**What it does**:
- Creates a synthetic test image (checkerboard + circle object)
- Simulates 3 types of camera noise:
  - **Gaussian noise**: Low light conditions (shot noise + read noise)
  - **Salt & pepper noise**: Pixel defects or electromagnetic interference
  - **Motion blur**: Camera or object movement during exposure
- Generates 4-panel comparison figure

**Output**: `../../diagrams/Ch1/camera_noise_comparison.png`

**Key Insight**: Noise significantly degrades edge detection and object recognition

---

### `lidar_visualization.py`
**Demonstrates**: How LiDAR sensors perceive environments

**What it does**:
- Simulates 2D LiDAR scan in a room with obstacles
- Models realistic sensor noise (±2cm Gaussian)
- Visualizes scan data in:
  - **Polar coordinates**: How LiDAR hardware outputs data
  - **Cartesian coordinates**: Bird's eye view for mapping/navigation

**Output**: `../../diagrams/Ch1/lidar_scan_visualization.png`

**Key Insight**: LiDAR provides geometric understanding but with inherent range uncertainty

---

### `physical_constraints.py`
**Demonstrates**: Quantitative impact of physical constraints on robot performance

**What it does**:
1. **Latency Impact Analysis**:
   - Calculates how far a robot travels during control latency
   - Shows why milliseconds matter (100ms @ 1 m/s = 10cm travel)

2. **Power Consumption Analysis**:
   - Compares runtime for different AI models (2W to 300W)
   - Demonstrates power-performance trade-off
   - Shows large models drain battery in <2 hours

3. **Sensor Noise Impact**:
   - Simulates 1000 LiDAR measurements with noise
   - Visualizes measurement distribution
   - Calculates 95% confidence interval

**Outputs**:
- `../../diagrams/Ch1/latency_impact.png`
- `../../diagrams/Ch1/power_consumption_analysis.png`
- `../../diagrams/Ch1/sensor_noise_distribution.png`

**Key Insight**: Physical constraints (latency, power, noise) fundamentally constrain Physical AI system design

---

## Expected Outputs

All scripts generate publication-quality figures saved to `../../diagrams/Ch1/`:

1. **camera_noise_comparison.png**: 4-panel comparison (clean, Gaussian, salt-pepper, motion blur)
2. **lidar_scan_visualization.png**: 2-panel (polar plot, Cartesian bird's eye view)
3. **latency_impact.png**: Line plot showing reaction distance vs robot speed for different latencies
4. **power_consumption_analysis.png**: Horizontal bar chart of AI model runtime
5. **sensor_noise_distribution.png**: Histogram of LiDAR measurements with true vs measured distance

---

## Troubleshooting

### Common Errors

**1. ModuleNotFoundError: No module named 'cv2'**
- **Cause**: OpenCV not installed or installed incorrectly
- **Fix**: `pip install opencv-python` (not `pip install cv2`)

**2. ImportError: DLL load failed (Windows)**
- **Cause**: Missing Visual C++ Redistributable
- **Fix**: Install VC++ Redist: https://aka.ms/vs/17/release/vc_redist.x64.exe

**3. Matplotlib backend errors (headless systems / SSH)**
- **Cause**: No display server available
- **Fix**: Set non-interactive backend before importing pyplot:
  ```python
  import matplotlib
  matplotlib.use('Agg')  # Non-interactive backend
  import matplotlib.pyplot as plt
  ```

**4. ValueError: cannot convert float NaN to integer**
- **Cause**: Division by zero or invalid intersection calculation in LiDAR simulation
- **Fix**: Code already includes checks; ensure NumPy version is 1.24+

**5. Figures don't display (plt.show() does nothing)**
- **Cause**: Running in background/SSH without X11 forwarding
- **Fix**: Figures are saved to files; check `../../diagrams/Ch1/` directory even if display fails

---

## Modifying Examples

### Exercise 1: Add Quantization Noise
Extend `camera_visualization.py` to simulate reduced bit depth:

```python
elif noise_type == 'quantization':
    # Reduce from 8-bit (256 levels) to 4-bit (16 levels)
    quantized = (image // 16) * 16
    return quantized.astype(np.uint8)
```

### Exercise 2: LiDAR Obstacle Detection
Extend `lidar_visualization.py` with clustering to detect obstacles:

```python
def detect_obstacles(x, y, distance_threshold=0.5, min_cluster_size=5):
    """Simple obstacle detection using distance clustering"""
    # Implement DBSCAN or distance-based clustering
    # Return list of obstacle centroids
    pass
```

---

## System Requirements

**Minimum**:
- Python 3.8+
- 2GB RAM
- 100MB disk space

**Recommended**:
- Python 3.10+
- 4GB RAM
- Ubuntu 22.04 or Windows 10+

**Dependencies**:
- numpy >= 1.24
- matplotlib >= 3.7
- opencv-python >= 4.8
- scipy >= 1.10 (for future exercises)

---

## Related Chapter Content

These code examples support concepts in:
- Section 1.4: Real-World Constraints (sensor noise, latency, power)
- Section 2.2: Visualizing Sensor Data (camera and LiDAR)
- Section 2.3: Physical Constraint Calculations (latency, power, noise)

For theoretical background, see `content/chapters/Ch1.md` sections 1.1-1.4.

---

## Contact / Issues

If code examples don't run as expected:
1. Verify Python version: `python --version` (should be 3.8+)
2. Verify dependencies: `pip list | grep -E "numpy|matplotlib|opencv"`
3. Check diagram output directory exists: `mkdir -p ../../diagrams/Ch1/`
4. Review troubleshooting section above

---

**Last Updated**: 2025-12-23
**Tested On**: Ubuntu 22.04, Python 3.10, numpy 1.24, matplotlib 3.7, opencv-python 4.8
