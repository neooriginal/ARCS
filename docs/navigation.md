# Navigation System

The ARCS navigation system provides robust obstacle detection, safety reflexes, and precision alignment tools for semi-autonomous operation. It leverages 360° LIDAR and computer vision analysis to guide the robot safely through environments and negotiate narrow passages.

## Core Components

### 1. 360° LIDAR System
The system uses a 360° USB LIDAR (LDROBOT LD19) for comprehensive distance sensing.

#### **A. Safety Layer (LIDAR)**
- **Sensor**: LD19 360° LIDAR (0.1m - 12m range)
- **Function**: Continuously measures distances in all directions simultaneously
- **Activation**: Only active during AI navigation (reduces noise during manual control)
- **Process**:
    - **STOP Condition**: If forward distance < `LIDAR_STOP_DISTANCE` (default 30cm), forward movement is blocked
    - **CAUTION Zone**: If forward distance < `LIDAR_WARN_DISTANCE` (default 80cm), the system flags a warning
    - **Directional Blocking**: Left/Right movement blocked based on side sensor readings (60°-120° arcs)

#### **B. Gap Detection**
- **Active Mode**: Used in **Precision Mode** for doorway navigation
- **Function**: Instantly analyzes 360° data to find passable gaps
- **Algorithm**:
    1. Scans the forward arc (±90° from center)
    2. Identifies depth contrast between walls (near) and openings (far)
    3. Finds the widest gap segment above minimum threshold
- **Output**: Gap center angle and width for AI alignment instructions

### 2. Safety Reflex
A low-level safety layer runs continuously during autonomous movement commands:
- **Monitoring**: The system checks 360° LIDAR data at 20Hz
- **Directional Safety**: Forward, Left, and Right movements blocked independently based on sector distances
- **Emergency Stop**: If an obstacle breaches the stop threshold in the movement direction, immediate brake

### 3. Precision Mode
Designed for finding and navigating narrow doorways:
- **360° LIDAR**: Instantly scans all directions - no robot rotation needed
- **Gap Finding**: Use `find_gap` tool to locate doorway openings
- **Usage**: Enable precision mode, call `find_gap`, turn to align, then drive forward

## Usage & Features
 
### 1. Holonomic Movement (Mecanum)
The robot is equipped with Mecanum wheels allowing for 3DoF movement:
- **Forward/Backward**
- **Rotate Left/Right**
- **Slide (Strafe) Left/Right**: Crucial for fine alignment without rotation

### 2. Precision Mode (Doorways)
Precision mode is essential for navigating doors:
1. **Enable**: The AI requests `enable_precision_mode()` or user toggles via API
2. **Find Gap**: Call `find_gap` to instantly analyze 360° LIDAR data
3. **Align**: Turn LEFT/RIGHT as instructed to center on the gap
4. **Drive**: Move forward through the detected opening

### 3. Approach Mode (Manipulation)
Used when the robot must interact with an object (touching distance):
- **Behavior**: 
    - 🛑 Speed is capped at **10%** for safety
    - 🛡️ "Stop distance" safety checks are relaxed to allow contact
- **Protocol**:
    1. Align from a distance using Holonomic slide
    2. Enable Approach Mode
    3. Move forward in small increments until interaction

### 4. Semantic Memory
The AI maintains a persistent mental map of the environment:
- **QR Context**: Passive scanning injects location data (e.g., "KITCHEN") into the AI's context
- **Persistent Notes**: The AI uses `save_note()` to record observations (e.g., "The hallway ends in a dead end"). These notes are retrieved in future sessions when relevant.

## Configuration

Key parameters in `config.json`:
- `LIDAR_PORT`: Serial port for the LD19 sensor (e.g., "/dev/ttyUSB0")
- `LIDAR_BAUD_RATE`: Communication speed (default: 230400)
- `LIDAR_STOP_DISTANCE`: Distance threshold for blocking movement (default: 30cm)
- `LIDAR_WARN_DISTANCE`: Distance threshold for caution warnings (default: 80cm)
- `OBSTACLE_THRESHOLD_RATIO`: Fraction of frame height for visual obstacle threshold (default: 0.875)
- `OBSTACLE_SOBEL_THRESHOLD`: Edge detection sensitivity (default: 30)
