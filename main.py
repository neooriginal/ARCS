"""RoboCrew Web Control & AI Agent"""

import signal
import sys
import threading
import time
import os
import logging
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
    create_look_left,
    create_look_right,
    create_look_center,
    create_look_down,
    create_look_up,
    create_end_task
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
    
    # 1. Initialize Robot System
    print("🔧 Initializing Robot System...")
    robot = RobotSystem()
    state.robot_system = robot
    
    # 2. Initialize AI Agent
    if robot.controller:
        print("🧠 Initializing AI Agent...")
        tools = [
            create_move_forward(robot.controller),
            create_move_backward(robot.controller),
            create_turn_left(robot.controller),
            create_turn_right(robot.controller),
            create_look_around(robot.controller, robot.camera),
            create_look_left(robot.controller),
            create_look_right(robot.controller),
            create_look_center(robot.controller),
            create_look_down(robot.controller),
            create_look_up(robot.controller),
            create_end_task()
        ]

        model_name = os.getenv("AI_MODEL", "openai/gpt-5.1") 
        
        try:
            agent = NavigationAgent(robot, model_name, tools)
            state.agent = agent
            print("✓ AI Agent ready")
        except Exception as e:
            print(f"⚠ AI Agent init failed: {e}")
    else:
        print("⚠ Robot controller not ready, AI disabled")

    # 3. Start Threads
    print("🔄 Starting background threads...", end=" ", flush=True)
    
    # Movement thread (manual control)
    threading.Thread(target=movement_loop, daemon=True).start()
    
    # AI Agent thread
    threading.Thread(target=agent_loop, daemon=True).start()
    print("✓")
    
    # 4. Start Web Server
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
