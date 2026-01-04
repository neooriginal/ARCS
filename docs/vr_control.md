# 🥽 VR Control

Control the robot arm using Quest 3 VR controllers.

> [!NOTE]
> **Compatibility**: Designed and tested for **Meta Quest 3 / 3S** using the standalone onboard browser (WebXR). Other headsets or PCVR setups are not guaranteed to work.

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
VR_WEBSOCKET_PORT = 8442  # VR WebSocket port
VR_TO_ROBOT_SCALE = 1.0   # Movement sensitivity
```

## Notes

- Uses PyBullet for inverse kinematics
- Camera feeds (Main & Arm) visible in VR at eye level

- Works with Quest 3 passthrough mode

## VLA Recording
To use VR for collecting LeRobot training data, see the [VLA Guide](vla_guide.md).
