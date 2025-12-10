# 🛡️ Safety Systems

The system uses a multi-layered architecture to ensure safe operation during autonomous control.

## 1. Active Reflex System
The system's "reflexes" operate faster than the AI loop to block unsafe actions proactively.
- **Wall Detection**: Halts forward movement if an obstacle is closer than the safety threshold (Y > 420).
- **Blindness Check**: Blocks movement if the camera is obstructed or viewing a featureless surface.

## 2. State Synchronization
Ensures the Web UI and AI Agent share the same "reality."
- **Shared Detector**: If the user sees "Blocked" on the HUD, the AI is programmatically prevented from moving forward.
- **Thread Safety**: Locks synchronization between the 30fps video stream and the asynchronous AI process.

## 3. Flicker Protection (Hysteresis)
Prevents sensor noise from confusing the AI.
- **Temporal Memory**: If an area is detected as blocked, it remains "blocked" in memory for ~0.5s even if the signal flickers. This creates a stable worldview.

## 4. Emergency Brake
- **Continuous Monitoring**: During long moves (e.g., "Forward 1m"), the camera is checked **10 times per second**.
- **Reaction**: Sudden obstacles trigger an immediate hardware stop.

## 5. Movement Constraints
- **No Double Backing**: The AI cannot reverse twice in a row, preventing blind backing into unknown areas.
