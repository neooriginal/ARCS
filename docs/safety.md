# 🛡️ Safety Systems

The system uses a multi-layered architecture to ensure safe operation during autonomous control.

## 1. Active Reflex System
The system's "reflexes" operate faster than the AI loop to block unsafe actions proactively.
- **Wall Detection**: Halts forward movement if an obstacle is closer than the safety threshold (set via `OBSTACLE_THRESHOLD_RATIO` in `config.py`).
- **Blindness Check**: Blocks movement if the camera is obstructed or viewing a featureless surface.

## 2. State Synchronization
Ensures the Web UI and AI Agent share the same "reality."
- **Shared Detector**: If the user sees "Blocked" on the HUD, the AI is programmatically prevented from moving forward.
- **Thread Safety**: Locks synchronization between the 30fps video stream and the asynchronous AI process.

## 3. Flicker Protection (Hysteresis)
Prevents sensor noise from confusing the AI.
- **Temporal Memory**: If an area is detected as blocked, it remains "blocked" in memory for ~0.5s even if the signal flickers. This creates a stable worldview.

## 4. Emergency Brake & Health Config
- **Continuous Monitoring**: During long moves (e.g., "Forward 1m"), the camera is checked **10 times per second**.
- **Reaction**: Sudden obstacles trigger an immediate hardware stop.
- **Darkness Prevention**: The AI measures scene brightness before every move. If `brightness < AI_MIN_BRIGHTNESS` (default 40), navigation is aborted to prevent hallucination.

## 5. Hardware Protection
- **Stall Detection**: Monitors motor loads in real-time. If a load exceeds `STALL_LOAD_THRESHOLD` (default 600), the affected motor group is disabled to prevent burnout.
- **Connection Safety**: A "Dead Man's Switch" monitors the control link. If no command is received for `REMOTE_TIMEOUT` (0.5s) while moving, the robot brakes automatically.

## 6. Movement Constraints
- **No Double Backing**: The AI cannot reverse twice in a row, preventing blind backing into unknown areas.
