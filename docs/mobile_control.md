# 📱 Mobile Control

ARCS now features a fully responsive mobile remote control interface, optimized for low-latency teleoperation from your smartphone or tablet.

## Features
- **Dual Virtual Joysticks**:
  - **Left Stick**: Robot drive (Omnidirectional WASD-style movement).
  - **Right Stick**: Head Pan/Tilt (Natural "mouse-look" controls).
- **Wheels-Only Design**: To ensure high performance and safety on small screens, mobile mode focuses exclusively on driving and vision. Arm/Gripper controls are disabled in the mobile layout.
- **Edge-to-Edge Video**: Optimized FPV (First Person View) layout with `object-fit: cover` to eliminate black bars and maximize screen usage.

## Usage
1. Open the dashboard on your mobile device: `http://<robot-ip>:5000/remote`.
2. The system automatically detects your device and switches to the touch-optimized layout.
3. No desktop or keyboard required.

> [!TIP]
> Use a landscape orientation for the best driving experience.
