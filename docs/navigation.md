# Navigation System

The ARCS navigation system provides robust obstacle detection, safety reflexes, and precision alignment tools for semi-autonomous operation. It leverages computer vision analysis to guide the robot safely through environments and negotiate narrow passages.

## Core Components

### 1. Hybrid LIDAR & Vision System
The system combines single-point LIDAR for robust safety with computer vision for guidance.

#### **A. Safety Layer (LIDAR)**
- **Sensor**: TF-Luna Single-Point LIDAR (0.2m - 8m range).
- **Function**: Continuously measures the distance directly in front of the robot.
- **Process**:
    - **STOP Condition**: If distance < `LIDAR_STOP_DISTANCE` (default 30cm), forward movement is hard-blocked.
    - **CAUTION Zone**: If distance < `LIDAR_WARN_DISTANCE` (default 80cm), the system flags a warning, but movement is allowed.

#### **B. Guidance Layer (Visual Gap Detection)**
- **Active Mode**: Only active during **Precision Mode**.
- **Function**: Scans the camera feed to find the "path of least resistance" (gaps).
- **Algorithm**:
    1. Extracts a horizontal strip from the center of the camera frame.
    2. Analyzes vertical pixel columns for intensity variance and brightness.
    3. Identifies the "smoothest" (low variance) and "darkest" (depth) area as the potential gap.
- **Output**: Visual target line on the HUD and rotation hints (`ROTATE LEFT` / `ROTATE RIGHT`) to align the robot with the gap.

### 2. Safety Reflex
A low-level safety layer runs continuously during autonomous movement commands.
- **Monitoring**: While moving, the system checks the LIDAR distance at 20Hz.
- **Emergency Stop**: If an obstacle breaches the stop threshold, the robot triggers an immediate brake.

### 3. Precision Mode
Designed for finding and navigating narrow doorways.
- **Hybrid Operation**:
    - **Vision**: Finds the doorway and guides alignment.
    - **LIDAR**: Ensures the path through the door is actually clear.
- **Usage**: Toggle Precision Mode, follow the yellow "TARGET" line until aligned, then drive forward.



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
