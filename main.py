"""
RoboCrew Web Control - Control your robot via browser with WASD + Mouse + Arm
Run: python main.py
Open: http://localhost:5000 (or your robot's IP)
"""

import signal
import sys
import threading
import os

from flask import Flask

from config import WEB_PORT, WHEEL_USB, HEAD_USB, ARM_CALIBRATION_PATH
from state import state
from camera import init_camera, release_camera
from movement import movement_loop, stop_movement
from arm import arm_controller
from routes import bp

# Import servo controller
from robocrew.robots.XLeRobot.servo_controls import ServoControler


def create_app():
    """Create and configure Flask application."""
    app = Flask(__name__)
    app.register_blueprint(bp)
    return app


def init_controller():
    """Initialize the servo controller with wheels, head, and arm."""
    print(f"🔧 Connecting servos ({WHEEL_USB}, {HEAD_USB})...", end=" ", flush=True)
    
    # Get calibration path (absolute)
    cal_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ARM_CALIBRATION_PATH))
    print(f"\n   Calibration: {cal_path}", end=" ", flush=True)
    
    try:
        # Initialize with arm enabled - arm uses same bus as wheels (IDs 1-6)
        state.controller = ServoControler(
            WHEEL_USB, 
            HEAD_USB,
            enable_arm=True,
            arm_calibration_path=cal_path
        )
        print("✓")
        
        # Check if arm was enabled
        if state.controller.arm_enabled:
            print("🦾 Arm connected ✓")
            state.arm_connected = True
            
            # Read current arm position
            try:
                pos = state.controller.get_arm_position()
                state.update_arm_positions(pos)
                arm_controller.set_from_current(pos)
            except Exception as e:
                print(f"⚠ Could not read arm: {e}")
        else:
            print("⚠ Arm not enabled")
        
        # Read current head position (don't move it!)
        print("📡 Reading current head position...", end=" ", flush=True)
        try:
            pos = state.controller.get_head_position()
            state.head_yaw = round(pos.get(7, 0), 1)
            state.head_pitch = round(pos.get(8, 0), 1)
            print(f"✓ (Yaw: {state.head_yaw}°, Pitch: {state.head_pitch}°)")
        except Exception as e:
            print(f"⚠ Could not read: {e}")
            state.head_yaw = 0
            state.head_pitch = 35
        
        return True
    except Exception as e:
        print(f"✗ Failed: {e}")
        state.controller = None
        state.last_error = f"Controller init failed: {e}"
        return False


def cleanup(signum=None, frame=None):
    """Graceful shutdown."""
    print("\n🛑 Shutting down...")
    state.running = False
    
    if state.controller:
        try:
            stop_movement()
            state.controller.disconnect()
            print("✓ Controller disconnected")
        except Exception as e:
            print(f"✗ Controller cleanup error: {e}")
    
    release_camera()
    sys.exit(0)


def main():
    """Main entry point."""
    print("=" * 50)
    print("🤖 RoboCrew Web Control")
    print("=" * 50)
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    # Initialize components
    init_camera()
    init_controller()
    
    # Start movement control thread
    print("🔄 Starting movement thread...", end=" ", flush=True)
    movement_thread = threading.Thread(target=movement_loop, daemon=True)
    movement_thread.start()
    print("✓")
    
    # Create and start web server
    app = create_app()
    
    print()
    print(f"🌐 Web server starting on http://0.0.0.0:{WEB_PORT}")
    print(f"   Open in browser to control the robot!")
    print()
    if state.last_error:
        print(f"⚠ Last error: {state.last_error}")
    print("Press Ctrl+C to stop")
    print("-" * 50)
    
    try:
        app.run(
            host='0.0.0.0',
            port=WEB_PORT,
            threaded=True,
            use_reloader=False,
            debug=False
        )
    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()
