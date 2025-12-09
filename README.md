# RoboCrew Control System

A robust robot control system featuring AI navigation, remote manipulation, and active safety systems.
Requires: [RoboCrew (Custom Branch)](https://github.com/neooriginal/RoboCrew/tree/custom)

## 🛡️ Safety Systems

This robot implements a multi-layered safety architecture to prevent collisions and ensuring reliable operation.

### 1. Active Reflex System
The "Lizard Brain" that runs faster than the AI. It proactively blocks unsafe actions.
- **Wall Detection**: Prevents forward movement if an obstacle is detected within the safety threshold (Y > 420).
- **Blindness Check**: Blocks movement if the camera view is obstructed or featureless (e.g., staring at a blank wall).
- **Green/Red Zones**: Visualizes safe paths (Green) and blocked areas (Red) on the HUD.

### 2. Discrepancy Synchronization
- **Shared Brain**: The Web UI and AI Agent share a single `ObstacleDetector` instance. This ensures that if the user sees "Blocked" on the screen, the AI also "knows" it is blocked.
- **Thread Safety**: Protected by locks to handle simultaneous access from the video stream (30fps) and the AI Agents.

### 3. Flicker Protection (Hysteresis)
- **Problem**: Visual noise can cause a wall to flicker between "Safe" and "Blocked", confusing the AI.
- **Solution**: A temporal memory buffer. If an area was blocked in the last ~0.5 seconds, it *stays* blocked. This creates a stable, cautious worldview.

### 4. Continuous Safety Monitoring (Emergency Brake)
- **Mechanism**: When the AI commands a long move (e.g., "Move Forward 1 meter"), the system doesn't just sleep. It checks the camera 10 times per second during the move.
- **Reaction**: If an obstacle suddenly appears (e.g., person walks in, or missed detection), the robot triggers an immediate **Emergency Stop**.

### 5. Backward Movement Constraints
- **Rule**: "No Double Backing". The AI is strictly prohibited from moving backward twice in a row.
- **Reason**: The robot has no rear camera. It may reverse once to unstick itself, but must then turn or check forward.

---

## 👁️ Visual Intelligence

### Thin Object Detection (Cable Problem)
- Uses **Top-N Average** logic instead of simple averaging.
- If the *closest* 2 scan lines detect an edge, it counts as an obstacle. This reliably detects thin horizontal cables that would otherwise be averaged out by floor noise.

### Tight Space Navigation (Guidance)
- **Gap Detection**: The vision system scans for the widest available "safe gap" between obstacles.
- **Auto-Guidance**: It calculates the center of this gap and provides textual guidance to the AI (e.g., "ALIGNMENT: Gap is LEFT. Turn LEFT slightly.").
- **Visual Target**: Displays a yellow "TARGET" line on the HUD to show where the robot should aim.

---

## 🎮 Setup & Calibration

**Calibrate Robot Head/Arm:**
```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/robot_acm0 --robot.id=xlerobot_arm
```
