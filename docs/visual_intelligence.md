# 👁️ Visual Intelligence

The robot employs a custom vision stack optimized for real-world clutter.

## 🕵️ Thin Object Detection
Standard vision often misses thin objects (cables, chair legs).
- **Top-N Logic**: Instead of averaging pixel columns, we track the *closest* edges.
- **Result**: Reliably detects power cords and table legs that would otherwise be invisible to the sensor.

## 🎯 Tight Space Navigation
Designed to thread the needle through doorways.
- **Gap Detection**: Scans for the widest available lateral gap.
- **Auto-Guidance**: Calculates the precise center of safe passage.
- **HUD Feedback**:
    - **Green**: Safe zone.
    - **Yellow Line**: Suggested target heading.
    - **Text**: Real-time steering hints (e.g., "Gap LEFT").
