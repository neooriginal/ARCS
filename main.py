"""ARCS Web Control & AI Agent"""

import signal
import sys
import threading
import time
import os
import logging
import subprocess
from dotenv import load_dotenv

load_dotenv()

from flask import Flask

try:
    from flask_socketio import SocketIO
    SOCKETIO_AVAILABLE = True
except ImportError:
    SOCKETIO_AVAILABLE = False
    # Will warn later if VR_ENABLED is True

from state import state
from movement import movement_loop, stop_movement
import routes
import tts
from core.robot_system import RobotSystem
from core.navigation_agent import NavigationAgent

from config import WEB_PORT, VR_ENABLED
from robots.xlerobot.tools import (
    create_move_forward, 
    create_move_backward, 
    create_turn_left, 
    create_turn_right, 
    create_look_around,
    create_slide_left,
    create_slide_right,
    create_end_task,
    create_enable_precision_mode,
    create_disable_precision_mode,
    create_save_note,
    create_enable_approach_mode,
    create_disable_approach_mode,
    create_speak,
    create_run_robot_policy
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
# Reduce noise from libraries
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('urllib3').setLevel(logging.ERROR)
logger = logging.getLogger(__name__)

def create_app():
    app = Flask(__name__)
    app.register_blueprint(routes.bp)
    app.config['SECRET_KEY'] = 'robocrew-vr-secret'
    return app

# Global SocketIO instance
socketio = None

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
    print("🤖 ARCS System Starting")
    print("=" * 50)
    
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    # Initialize Robot System
    print("🔧 Initializing Robot System...")
    robot = RobotSystem()
    state.robot_system = robot
    
    # Initialize TTS
    tts.init()
    
    # Initialize AI Agent
    if robot.controller:
        print("💡 Initializing AI Agent...")
        # Minimal tools - no individual camera controls to avoid confusion
        tools = [
            create_move_forward(robot.controller),
            create_move_backward(robot.controller),
            create_turn_left(robot.controller),
            create_turn_right(robot.controller),
            create_slide_left(robot.controller),
            create_slide_right(robot.controller),
            create_look_around(robot.controller, robot.camera),
            create_end_task(),
            create_enable_precision_mode(),
            create_disable_precision_mode(),
            create_save_note(),
            create_enable_approach_mode(),
            create_disable_approach_mode(),
            create_speak(),
            create_run_robot_policy()
        ]

        model_name = os.getenv("AI_MODEL", "openai/gpt-5.2") 
        
        try:
            agent = NavigationAgent(robot, model_name, tools)
            state.agent = agent
            print("✓ AI Agent ready")
        except Exception as e:
            logger.warning(f"AI Agent init failed: {e}")
    else:
        logger.warning("Robot controller not ready, AI disabled")

    # Start Threads
    print("🔄 Starting background threads...", end=" ", flush=True)
    
    # Movement thread (manual control)
    threading.Thread(target=movement_loop, daemon=True).start()
    
    # AI Agent thread
    threading.Thread(target=agent_loop, daemon=True).start()
    

    
    # Initialize VR Control (if enabled)
    vr_controller = None
    arm_available = robot.controller and hasattr(robot.controller, 'arm_enabled') and robot.controller.arm_enabled
    
    if VR_ENABLED and SOCKETIO_AVAILABLE and robot.controller:
        if arm_available:
            print("🥽 Initializing VR Control...")
            try:
                from vr_arm_controller import VRArmController
                import vr_arm_controller as vr_module
                
                # Movement callback for joystick
                def vr_movement_callback(forward, lateral, rotation):
                    if robot.controller:
                        robot.controller.set_velocity_vector(forward, lateral, rotation)
                
                vr_controller = VRArmController(robot.controller, vr_movement_callback)
                vr_module.vr_arm_controller = vr_controller
                print("✓ VR Control initialized")
            except Exception as e:
                logger.warning(f"VR Control init failed: {e}")
                logger.debug(sys.exc_info()) # Log trace to debug
        else:
            logger.warning("VR Control disabled (arm not enabled on controller)")
    elif VR_ENABLED and not SOCKETIO_AVAILABLE:
        logger.warning("VR Control disabled (flask-socketio not installed)")
        print("   Install with: pip install flask-socketio")
    
    # TTS Startup Announcement
    tts.speak("System ready")
    
    # Start Web Server
    app = create_app()
    
    # Setup Socket.IO for VR if available
    global socketio
    if SOCKETIO_AVAILABLE:
        socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
        
        # Register VR socket events
        if vr_controller:
            @socketio.on('connect')
            def handle_connect():
                vr_controller.vr_handler.on_connect()
            
            @socketio.on('disconnect')
            def handle_disconnect():
                vr_controller.vr_handler.on_disconnect()
            
            @socketio.on('vr_connect')
            def handle_vr_connect():
                print("🥽 VR client connected")
            
            @socketio.on('vr_data')
            def handle_vr_data(data):
                vr_controller.vr_handler.on_vr_data(data)
    
    print()
    print(f"🌐 http://localhost:{WEB_PORT}")
    print(f"🥽 VR: http://localhost:{WEB_PORT}/vr")
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
        if socketio:
            socketio.run(app, host='0.0.0.0', port=WEB_PORT, debug=False, allow_unsafe_werkzeug=True)
        else:
            app.run(host='0.0.0.0', port=WEB_PORT, threaded=True, use_reloader=False, debug=False)
    except KeyboardInterrupt:
        cleanup()

if __name__ == "__main__":
    main()
