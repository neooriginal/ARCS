# 🥽 VR Control

Control the robot arm using Quest 3 VR controllers.

## Usage

1. Start the robot: `python main.py`
2. On Quest 3: Open browser → `http://<robot-ip>:5000/vr`
3. Click "Start VR" button

## Controls

| Input | Action |
|-------|--------|
| **Right Grip (hold)** | Arm follows controller position |
| **Right Trigger** | Close gripper |
| **Left Thumbstick** | Move robot base |
| **Controller rotation** | Wrist roll/flex |

## Config

In `config.py`:
```python
VR_ENABLED = True         # Enable/disable VR
VR_WEBSOCKET_PORT = 5000  # Same as web port (Socket.IO)
VR_TO_ROBOT_SCALE = 1.0   # Movement sensitivity
```

## Notes

- Uses PyBullet for inverse kinematics
- Camera feed visible in VR at eye level
- Works with Quest 3 passthrough mode
