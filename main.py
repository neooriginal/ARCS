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

from config import WEB_PORT, VR_ENABLED, CAMERA_PORT
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

from core.log_handler import CircularLogHandler

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

# Add Circular Log Handler
log_handler = CircularLogHandler()
logging.getLogger().addHandler(log_handler)
state.log_handler = log_handler

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
    
    # Clean up VR Kinematics (PyBullet)
    try:
        from core.vr_kinematics import vr_kinematics
        vr_kinematics.cleanup()
    except Exception:
        pass
    
    import tts
    tts.shutdown()
    
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    # Initialize Robot System
    logger.info("Initializing Robot System...")
    robot = RobotSystem()
    state.robot_system = robot
    
    # Initialize TTS
    logger.info("Initializing TTS...")
    tts.init()
    
    # Initialize AI Agent
    if robot.controller:
        logger.info("Initializing AI Agent...")
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
            logger.info("AI Agent ready")
        except Exception as e:
            logger.warning(f"AI Agent init failed: {e}")
    else:
        logger.warning("Robot controller not ready, AI disabled")

    # Start Threads
    logger.info("Starting background threads...")
    
    # Movement thread (manual control)
    threading.Thread(target=movement_loop, daemon=True).start()
    
    # AI Agent thread
    threading.Thread(target=agent_loop, daemon=True).start()
    
    
    # Initialize VR Control (if enabled)
    vr_controller = None
    
    # Check if camera exists (vital for VR)
    camera_exists = False
    if isinstance(CAMERA_PORT, str) and os.path.exists(CAMERA_PORT):
        camera_exists = True
    elif isinstance(CAMERA_PORT, int):
         # If integer index, check typical usage
         camera_exists = os.path.exists(f"/dev/video{CAMERA_PORT}")
    
    if VR_ENABLED and SOCKETIO_AVAILABLE:
        if not camera_exists:
             logger.warning(f"VR Control disabled: Camera {CAMERA_PORT} not found")
        else:
            logger.info("Initializing VR Control...")
        try:
            from vr_arm_controller import VRArmController
            import vr_arm_controller as vr_module
            
            # Movement callback for joystick
            def vr_movement_callback(forward, lateral, rotation):
                if state.robot_system and state.robot_system.controller:
                    state.robot_system.controller.set_velocity_vector(forward, lateral, rotation)
            
            # Late-binding of hardware controller
            vr_controller = VRArmController(None, vr_movement_callback)
            vr_module.vr_arm_controller = vr_controller
            logger.info("VR Control initialized")
        except Exception as e:
            logger.warning(f"VR Control init failed: {e}")
    elif VR_ENABLED and not SOCKETIO_AVAILABLE:
        logger.warning("VR Control disabled (flask-socketio not installed)")
    
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
                logger.info("VR client connected")
            
            @socketio.on('vr_data')
            def handle_vr_data(data):
                # Late binding check for controller
                if vr_controller.controller is None and state.robot_system and state.robot_system.controller:
                    vr_controller.controller = state.robot_system.controller
                
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
                logger.info("Display opened on main screen")
            except FileNotFoundError:
                try:
                    # Fallback to firefox
                    subprocess.Popen(['firefox', '--kiosk', display_url], env=env,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    logger.info("Display opened on main screen")
                except FileNotFoundError:
                    logger.warning("Could not auto-open display (no browser found)")
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
