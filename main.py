"""RoboCrew Web Control"""

import signal
import sys
import threading
import os

# Add local RoboCrew source to path (use local instead of pip-installed module)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'RoboCrew', 'src'))

from flask import Flask

from config import WEB_PORT, WHEEL_USB, HEAD_USB
from state import state
from camera import init_camera, release_camera
from movement import movement_loop, stop_movement
from arm import arm_controller
from routes import bp
from robocrew.robots.XLeRobot.servo_controls import ServoControler


def create_app():
    app = Flask(__name__)
    app.register_blueprint(bp)
    return app


def init_controller():
    print(f"🔧 Connecting servos ({WHEEL_USB}, {HEAD_USB})...", end=" ", flush=True)
    
    try:
        state.controller = ServoControler(
            WHEEL_USB, 
            HEAD_USB,
            enable_arm=True,
        )
        print("✓")
        
        if state.controller.arm_enabled:
            print("🦾 Arm connected ✓")
            state.arm_connected = True
            try:
                pos = state.controller.get_arm_position()
                state.update_arm_positions(pos)
                arm_controller.set_from_current(pos)
            except Exception as e:
                print(f"⚠ Could not read arm: {e}")
        
        print("📡 Reading head position...", end=" ", flush=True)
        try:
            pos = state.controller.get_head_position()
            state.head_yaw = round(pos.get(7, 0), 1)
            state.head_pitch = round(pos.get(8, 0), 1)
            print(f"✓ (Yaw: {state.head_yaw}°, Pitch: {state.head_pitch}°)")
        except Exception as e:
            print(f"⚠ {e}")
            state.head_yaw = 0
            state.head_pitch = 35
        
        return True
    except Exception as e:
        print(f"✗ Failed: {e}")
        state.controller = None
        state.last_error = str(e)
        return False


def cleanup(signum=None, frame=None):
    print("\n🛑 Shutting down...")
    state.running = False
    
    if state.controller:
        try:
            stop_movement()
            state.controller.disconnect()
            print("✓ Disconnected")
        except Exception as e:
            print(f"✗ {e}")
    
    release_camera()
    sys.exit(0)


def main():
    print("=" * 50)
    print("🤖 RoboCrew Web Control")
    print("=" * 50)
    
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    init_camera()
    init_controller()
    
    print("🔄 Starting movement thread...", end=" ", flush=True)
    threading.Thread(target=movement_loop, daemon=True).start()
    print("✓")
    
    app = create_app()
    
    print()
    print(f"🌐 http://0.0.0.0:{WEB_PORT}")
    print("Press Ctrl+C to stop")
    print("-" * 50)
    
    try:
        app.run(host='0.0.0.0', port=WEB_PORT, threaded=True, use_reloader=False, debug=False)
    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()
