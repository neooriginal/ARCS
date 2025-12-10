"""RoboCrew Web Control & AI Agent"""

import signal
import sys
import threading
import time
import os
import logging
import subprocess
from dotenv import load_dotenv

load_dotenv()

# Add local RoboCrew source to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'RoboCrew', 'src'))

from flask import Flask

from config import WEB_PORT
from state import state
from movement import movement_loop, stop_movement
from routes import bp

from robocrew.core.robot_system import RobotSystem
from robocrew.core.navigation_agent import NavigationAgent
from robocrew.robots.XLeRobot.tools import (
    create_move_forward, 
    create_move_backward, 
    create_turn_left, 
    create_turn_right, 
    create_look_around,
    create_look_around,
    create_end_task,
    create_check_alignment
)

# Configure logging - reduce verbosity
logging.basicConfig(level=logging.WARNING, format='%(asctime)s - %(levelname)s - %(message)s')
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('urllib3').setLevel(logging.ERROR)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

def create_app():
    app = Flask(__name__)
    app.register_blueprint(bp)
    return app

def agent_loop():
    """Background thread for AI Agent."""
    logger.info("AI Agent loop started")
    while state.running:
        if state.ai_enabled and state.agent:
            try:
                status = state.agent.step()
                state.add_ai_log(status)
            except Exception as e:
                logger.error(f"Agent step error: {e}")
                state.add_ai_log(f"Error: {e}")
                state.ai_enabled = False # Safety disable
        time.sleep(0.1)

def cleanup(signum=None, frame=None):
    print("\n🛑 Shutting down...")
    state.running = False
    
    if state.robot_system:
        state.robot_system.cleanup()
    
    sys.exit(0)

def main():
    print("=" * 50)
    print("🤖 RoboCrew System Starting")
    print("=" * 50)
    
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    # Initialize Robot System
    print("🔧 Initializing Robot System...")
    robot = RobotSystem()
    state.robot_system = robot
    
    # Initialize AI Agent (using new robust init)
    print("🧠 Initializing AI Agent...")
    if state.init_agent():
        print("✓ AI Agent ready")
    else:
        print("⚠ AI Agent deferred (waiting for controller/camera)")

    # Start Threads
    print("🔄 Starting background threads...", end=" ", flush=True)
    
    # Movement thread (manual control)
    threading.Thread(target=movement_loop, daemon=True).start()
    
    # AI Agent thread
    threading.Thread(target=agent_loop, daemon=True).start()
    print("✓")
    
    # Start Web Server
    app = create_app()
    
    print()
    print(f"🌐 http://0.0.0.0:{WEB_PORT}")
    print(f"📺 Display: http://localhost:{WEB_PORT}/display")
    print("Press Ctrl+C to stop")
    print("-" * 50)
    
    # Auto-open display on Raspberry Pi's physical screen (works over SSH)
    if os.getenv('AUTO_OPEN_DISPLAY', 'true').lower() == 'true':
        def open_display():
            import time
            time.sleep(2)  # Wait for server to start
            display_url = f'http://localhost:{WEB_PORT}/display'
            env = os.environ.copy()
            env['DISPLAY'] = ':0'  # Target the main display
            try:
                # Try chromium-browser first (common on Raspberry Pi)
                subprocess.Popen(
                    ['chromium-browser', '--kiosk', '--noerrdialogs', '--disable-infobars', display_url],
                    env=env,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                print("📺 Display opened on main screen")
            except FileNotFoundError:
                try:
                    # Fallback to firefox
                    subprocess.Popen(['firefox', '--kiosk', display_url], env=env,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    print("📺 Display opened on main screen")
                except FileNotFoundError:
                    print("⚠ Could not auto-open display (no browser found)")
        threading.Thread(target=open_display, daemon=True).start()
    
    try:
        app.run(host='0.0.0.0', port=WEB_PORT, threaded=True, use_reloader=False, debug=False)
    except KeyboardInterrupt:
        cleanup()

if __name__ == "__main__":
    main()
