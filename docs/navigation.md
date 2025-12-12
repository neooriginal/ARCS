# Navigation System

The ARCS navigation system provides robust obstacle detection, safety reflexes, and precision alignment tools for semi-autonomous operation. It leverages computer vision analysis to guide the robot safely through environments and negotiate narrow passages.

## Core Components

### 1. Vision-Based Obstacle Detection
The system analyzes video frames in real-time to detect physical obstacles.
- **Preprocessing**: Applies `Bilateral Filtering` to reduce noise while preserving edges.
- **Edge Detection**: Uses `Canny Edge Detection` to identify structural boundaries.
- **Column Scanning**: The frame is scanned vertically to find the lowest (closest) edge points, creating a depth-map approximation where lower pixels in the frame (higher Y-values) represent closer obstacles.

### 2. Safety Reflex
A low-level safety layer runs continuously during autonomous movement commands.
- **Monitoring**: While moving, the system checks the obstacle detector at 10Hz.
- **Emergency Stop**: If an obstacle appears within the critical threshold (default Y > 420), the robot triggers an immediate "Safety Reflex" brake, canceling the current action to prevent collision.

### 3. Precision Mode
Designed for navigating narrow doorways or tight gaps where standard safety thresholds would prevent movement.
- **Adaptive Thresholds**: When enabled, side safety margins are relaxed to allow the robot to pass close to door frames.
- **Gap Alignment**: The system identifies the widest contiguous gap in the field of view.
- **Visual Guidance**: Overlays alignment targets and text instructions (e.g., "ALIGNMENT: TARGET LEFT") on the video feed to assist the operator or AI agent in aligning perfectly with the opening.

### 4. SLAM & Mapping
The system includes a lightweight monocular SLAM (Simultaneous Localization and Mapping) module (`SimpleSLAM`) to estimate the robot's trajectory and build a local occupancy grid.
- **Visual Odometry**: Uses Lucas-Kanade optical flow to track features between frames. It estimates:
    - **Rotation**: From horizontal pixel shifts.
    - **Translation**: From vertical pixel shifts (assuming ground plane motion).
- **Occupancy Map**: Projects vision-detected obstacles onto a 2D top-down grid map.
    - **Input**: Video frames (640x480).
    - **Output**: 800x800 pixel grid map (5cm/pixel resolution) with the robot centered.
    - **Update**: As the robot moves, new obstacle observations are drawn onto the map, and the robot's pose path is traced.

## Usage

### Operating Precision Mode
Precision mode is essential for navigating doors.
1. **Enable**: call explicit tool or toggle via API.
2. **Align**: Rotate the robot until the yellow "TARGET" line turns green and the status reads "PERFECT".
3. **Drive**: Move forward through the gap. The safety reflex remains active but with adjusted sensitivity.

### Calibration
Key parameters in `obstacle_detection.py`:
- `obstacle_threshold_y` (Default: 420): The Y-coordinate limit for forward obstacles. Increase to allow closer approach (max 480).
- `min_edge_pixels` (Default: 200): Minimum visual detail required to consider the camera "active" (prevents movement if camera is covered/dark).
