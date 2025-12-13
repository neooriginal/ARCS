# Visual Intelligence System

The RoboCrew Control System uses a modular computer vision stack to understand its environment. This document details the subsystems for obstacle detection, precision maneuvering, and semantic context (QR codes).

## 1. Obstacle Detection System
**File**: `obstacle_detection.py`

The primary safety layer is a deterministic vision algorithm that prevents collisions.
*   **Edge Detection**: We use Canny Edge Detection (OpenCV) to identify high-frequency changes in the image (walls, furniture).
*   **Column Scanning**: The image is scanned in vertical columns to find the lowest (closest) edge pixel.
*   **Safety Zones**: The image is divided into `Left`, `Forward`, and `Right` zones.
*   **Reflex Action**: If the average distance to obstacles in any zone is below a threshold (too close), that direction is marked as **BLOCKED**. The `NavigationAgent` is physically unable to command a move in a blocked direction.

## 2. Precision Mode (Doorway Navigation)
**File**: `obstacle_detection.py`

Standard obstacle avoidance is too conservative for narrow gaps like doorways. "Precision Mode" relaxes these constraints and adds guidance.
*   **Activation**: The AI must explicitly call `enable_precision_mode()` before approaching a narrow gap.
*   **Gap Detection**: 
    - The system identifies "passable" columns (where obstacles are far away).
    - It groups these columns into gaps.
    - **Smart Selection**: It selects the best gap based on a balance of **Width** and **Centrality** (`Score = Width - 1.2 * DistToCenter`).
*   **Visual Guidance**:
    - A **Yellow Line** is drawn on the camera feed pointing to the target gap center.
    - **Smoothing**: The line position is smoothed (filtered) to prevent jitter and erratic steering.
*   **Close-Range Behaviors**:
    - **Rotation Hints**: If blocked, the system hints: `ROTATE LEFT/RIGHT to align`.
    - **Blind Commit**: When very close (`c_fwd > 420`), sensors are masked to prevent the door frame itself from triggering a stop. The system outputs `BLIND COMMIT`.

## 3. QR Code Context System
**File**: `qr_scanner.py`

The robot passively scans for QR codes to gain semantic understanding of its customized environment.

### Format
QR codes should follow this strictly to be visualized correctly:
```text
LOCATION-TITLE: Optional longer description or data...
```
*   **Example 1**: `KITCHEN: Area with fridge and stove`
*   **Example 2**: `CHARGING_DOCK` (Title only is valid too)

### Capabilities
1.  **Passive Detection**: The system scans every frame.
2.  **Context Injection**: When a **NEW** code is seen:
    *   The `Title` and `Description` are extracted.
    *   The AI receives a high-priority system message: `CONTEXT UPDATE: Detected marker 'KITCHEN' at x=1.5, y=3.2`.
    *   **De-duplication**: Unique codes are reported only **once** per session to prevent flooding the AI's context window.
3.  **Visualization**:
    *   **Green Bounding Polygon**: A green outline is drawn connecting the 4 corners of the detected QR code.
    *   **Label**: The `LOCATION-TITLE` is rendered in green text just below the polygon.
