"""ARCS Web Control & AI Agent"""

import signal
import sys
import threading
import time
import os
import logging
import subprocess
from typing import Optional, NoReturn

from dotenv import load_dotenv
from flask import Flask

try:
    from flask_socketio import SocketIO
    SOCKETIO_AVAILABLE = True
except ImportError:
    SOCKETIO_AVAILABLE = False

from state import state
from movement import movement_loop
import routes
import tts
from core.robot_system import RobotSystem
from core.navigation_agent import NavigationAgent
from core.config_manager import get_config
from core.log_handler import CircularLogHandler
from robots.xlerobot.tools import (
    create_move_forward, create_move_backward,
    create_turn_left, create_turn_right,
    create_look_around, create_slide_left, create_slide_right,
    create_end_task, create_enable_precision_mode, create_disable_precision_mode,
    create_save_note, create_enable_approach_mode, create_disable_approach_mode,
    create_speak, create_run_robot_policy
)

load_dotenv()

WEB_PORT = get_config("WEB_PORT")
CAMERA_PORT = get_config("CAMERA_PORT")

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('urllib3').setLevel(logging.ERROR)
logger = logging.getLogger(__name__)

# Add Circular Log Handler
log_handler = CircularLogHandler()
logging.getLogger().addHandler(log_handler)
state.log_handler = log_handler

class WerkzeugErrorFilter(logging.Filter):
    def filter(self, record):
        return "write() before start_response" not in record.getMessage()

logging.getLogger('werkzeug').addFilter(WerkzeugErrorFilter())

def create_app() -> Flask:
    app = Flask(__name__)
    app.register_blueprint(routes.bp)
    app.config['SECRET_KEY'] = 'ARCS-vr-secret'
    app.config['TEMPLATES_AUTO_RELOAD'] = True
    return app

# Global SocketIO instance
socketio: Optional[SocketIO] = None
vr_controller = None

def agent_loop() -> None:
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
                state.ai_enabled = False
        time.sleep(0.1)

def init_vr_control() -> None:
    """Initialize VR controller (called from deferred_init)."""
    global vr_controller

    if not SOCKETIO_AVAILABLE:
        logger.warning("VR Control disabled (flask-socketio not installed)")
        return

    camera_exists = False
    if isinstance(CAMERA_PORT, str) and os.path.exists(CAMERA_PORT):
        camera_exists = True
    elif isinstance(CAMERA_PORT, int):
        camera_exists = os.path.exists(f"/dev/video{CAMERA_PORT}")

    if not camera_exists:
        logger.warning(f"VR Control disabled: Camera {CAMERA_PORT} not found")
        return

    logger.info("Initializing VR Control...")
    try:
        from vr_arm_controller import VRArmController
        import vr_arm_controller as vr_module

        def vr_movement_callback(forward: float, lateral: float, rotation: float) -> None:
            if state.robot_system and state.robot_system.controller:
                state.robot_system.controller.set_velocity_vector(forward, lateral, rotation)

        vr_controller = VRArmController(None, vr_movement_callback)
        vr_module.vr_arm_controller = vr_controller
        logger.info("VR Control initialized")
    except Exception as e:
        logger.warning(f"VR Control init failed: {e}")

def cleanup(signum=None, frame=None) -> NoReturn:
    print("\n🛑 Shutting down...")
    state.running = False

    if state.robot_system:
        state.robot_system.cleanup()

    try:
        from core.vr_kinematics import vr_kinematics
        vr_kinematics.cleanup()
    except Exception:
        pass
    
    # Cleanup lidar
    if state.lidar:
        try:
            state.lidar.disconnect()
        except Exception:
            pass

    import tts
    tts.shutdown()

    sys.exit(0)

def _setup_agent(robot: RobotSystem) -> None:
    if not robot.controller:
        logger.warning("Robot controller not ready, AI disabled")
        return

    logger.info("Initializing AI Agent...")
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

def _deferred_init() -> None:
    logger.info("Initializing Robot System...")
    robot = RobotSystem()
    state.robot_system = robot

    logger.info("Initializing TTS...")
    tts.init()

    _setup_agent(robot)

    threading.Thread(target=movement_loop, daemon=True).start()
    threading.Thread(target=agent_loop, daemon=True).start()

    init_vr_control()
    
    # Initialize lidar sensor (auto-detects connection)
    try:
        from core.lidar import init_lidar
        if init_lidar():
            logger.info("Lidar sensor connected")
        else:
            logger.debug("Lidar sensor not available")
    except Exception as e:
        logger.debug(f"Lidar init skipped: {e}")

    tts.speak("System ready")
    logger.info("Hardware initialization complete")

def _open_display_browser() -> None:
    if os.getenv('AUTO_OPEN_DISPLAY', 'true').lower() != 'true':
        return

    time.sleep(2)
    display_url = f'http://localhost:{WEB_PORT}/display'
    env = os.environ.copy()
    env['DISPLAY'] = ':0'

    for browser_cmd in [
        ['chromium-browser', '--kiosk', '--noerrdialogs', '--disable-infobars', display_url],
        ['firefox', '--kiosk', display_url]
    ]:
        try:
            subprocess.Popen(
                browser_cmd,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            logger.info(f"Display opened on main screen using {browser_cmd[0]}")
            return
        except FileNotFoundError:
            continue

    logger.warning("Could not auto-open display (no browser found)")

def main() -> None:
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    threading.Thread(target=_deferred_init, daemon=True).start()
    threading.Thread(target=_open_display_browser, daemon=True).start()

    app = create_app()

    global socketio
    if SOCKETIO_AVAILABLE:
        socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

        @socketio.on('connect')
        def handle_connect():
             # Basic Auth Check
            from flask import request
            from flask_socketio import disconnect
            from core.auth import verify_token
            
            token = request.cookies.get('auth_token')
            if not token or not verify_token(token):
                disconnect()
                return

            if vr_controller:
                vr_controller.vr_handler.on_connect()

        @socketio.on('disconnect')
        def handle_disconnect():
            if vr_controller:
                vr_controller.vr_handler.on_disconnect()

        @socketio.on('vr_connect')
        def handle_vr_connect():
            logger.info("VR client connected")

        @socketio.on('vr_data')
        def handle_vr_data(data):
            if not vr_controller:
                return
            if vr_controller.servo_controller is None and state.robot_system and state.robot_system.controller:
                vr_controller.servo_controller = state.robot_system.controller

            vr_controller.vr_handler.on_vr_data(data)

    print()
    print(f"🌐 http://localhost:{WEB_PORT}")
    print(f"🥽 VR: http://localhost:{WEB_PORT}/vr")
    print(f"📺 Display: http://localhost:{WEB_PORT}/display")
    print("Press Ctrl+C to stop")
    print("-" * 50)

    try:
        if socketio:
            socketio.run(app, host='0.0.0.0', port=WEB_PORT, debug=False, allow_unsafe_werkzeug=True)
        else:
            app.run(host='0.0.0.0', port=WEB_PORT, threaded=True, use_reloader=False, debug=False)
    except KeyboardInterrupt:
        cleanup()

if __name__ == "__main__":
    main()
