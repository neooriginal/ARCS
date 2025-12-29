# Navigation System

The ARCS navigation system provides robust obstacle detection, safety reflexes, and precision alignment tools for semi-autonomous operation. It leverages computer vision analysis to guide the robot safely through environments and negotiate narrow passages.

## Core Components

### 1. Vision-Based Obstacle Detection
The system analyzes video frames in real-time to detect physical obstacles.
- **Preprocessing**: Converts to grayscale and applies `Gaussian Blur` to reduce noise.
- **Edge Detection**: Uses **Gradient Magnitude** (Sobel-X + Sobel-Y) to detect edges in *any* direction. This allows detection of both vertical obstacles (walls, legs) and horizontal boundaries (baseboards).
- **Column Scanning**: The frame is scanned vertically to find the lowest (closest) edge points, creating a depth-map approximation where lower pixels in the frame (higher Y-values) represent closer obstacles.

### 2. Safety Reflex
A low-level safety layer runs continuously during autonomous movement commands.
- **Monitoring**: While moving, the system checks the obstacle detector at 10Hz.
- **Emergency Stop**: If an obstacle appears within the critical threshold (calculated from `OBSTACLE_THRESHOLD_RATIO` in `config.py`), the robot triggers an immediate "Safety Reflex" brake, canceling the current action to prevent collision.

### 3. Precision Mode
Designed for navigating narrow doorways or tight gaps where standard safety thresholds would prevent movement.
- **Adaptive Thresholds**: When enabled, side safety margins are relaxed to allow the robot to pass close to door frames.
- **Gap Alignment**: The system identifies the widest contiguous gap in the field of view.
- **Visual Guidance**: Overlays alignment targets and text instructions (e.g., "ALIGNMENT: TARGET LEFT") on the video feed to assist the operator or AI agent in aligning perfectly with the opening.



## Usage & Features
 
### 1. Holonomic Movement (Mecanum)
The robot is equipped with Mecanum wheels allowing for 3DoF movement:
- **Forward/Backward**
- **Rotate Left/Right**
- **Slide (Strafe) Left/Right**: Crucial for fine alignment without rotation.

### 2. Precision Mode (Doorways)
Precision mode is essential for navigating doors.
1. **Enable**: The AI requests `enable_precision_mode()` or user toggles via API.
2. **Align**: Rotate until the yellow "TARGET" line turns green ("PERFECT").
3. **Drive**: The safety reflex automatically adjusts bounds to allow passing through gaps > 35cm.

### 3. Approach Mode (Manipulation)
Used when the robot must interact with an object (touching distance).
- **Behavior**: 
    - 🛑 Speed is capped at **10%** for safety.
    - 🛡️ "Stop distance" safety checks are relaxed to allow contact.
- **Protocol**:
    1. Align from a distance using Holonomic slide.
    2. Enable Approach Mode.
    3. Move forward in small increments until interaction.

### 4. Semantic Memory
The AI maintains a persistent mental map of the environment:
- **QR Context**: Passive scanning injects location data (e.g., "KITCHEN") into the AI's context.
- **Persistent Notes**: The AI uses `save_note()` to record observations (e.g., "The hallway ends in a dead end"). These notes are retrieved in future sessions when relevant.

### Calibration
Key parameters in `config.py`:
- `OBSTACLE_THRESHOLD_RATIO` (Default: 0.875): Fraction of frame height for forward obstacle threshold. Higher values allow closer approach (max 1.0).
- `OBSTACLE_SOBEL_THRESHOLD` (Default: 30): Edge detection sensitivity. Lower values detect more edges (more sensitive).
