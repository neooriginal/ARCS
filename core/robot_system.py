import threading
import logging
from typing import Optional

from state import state
from robots import load_robot
from robots.base import BaseRobot
from core.config_manager import get_config

WHEEL_USB = get_config("WHEEL_USB")
ROBOT_TYPE = get_config("ROBOT_TYPE", "xlerobot")

logger = logging.getLogger(__name__)

class RobotSystem:
    def __init__(self):
        self.robot: Optional[BaseRobot] = None
        self.camera = None
        self.camera_lock = threading.Lock()
        self.running = True
        
        # Start hardware initialization in background to allow UI to load
        threading.Thread(target=self._init_hardware, daemon=True).start()
        
        state.robot_system = self
        
    def _init_hardware(self):
        """Background initialization sequence."""
        logger.info("Starting hardware initialization sequence...")
        self._init_camera()
        self._init_robot()
        logger.info("Hardware initialization complete")

    def _init_camera(self):
        try:
            from camera import init_camera
            if init_camera():
                self.camera = state.camera
                logger.info("Camera initialized via camera module")
            else:
                logger.error("Failed to initialize camera module")
                self.camera = None
        except ImportError:
            logger.error("Camera module not found or dependencies missing")
            self.camera = None
        except Exception as e:
            logger.error(f"Camera init error: {e}")
            self.camera = None

    def _init_robot(self):
        """Initialize robot driver using factory."""
        logger.info(f"Loading robot type '{ROBOT_TYPE}'...")
        try:
            self.robot = load_robot(
                ROBOT_TYPE,
                wheel_usb=WHEEL_USB,
                enable_arm=True,
            )
            self.robot.connect()
            
            # Update state for backwards compatibility
            if hasattr(self.robot, 'controller'):
                state.controller = self.robot.controller
            
            # Initial readings
            if self.robot.has_arm:
                state.arm_connected = True
                try:
                    pos = self.robot.get_arm_joints()
                    state.update_arm_positions(pos)
                except Exception as e:
                    logger.warning(f"Could not read arm: {e}")
            

                    
            logger.info(f"Robot '{self.robot.name}' connected successfully")
            
            # Initialize AI agent now that controller is ready
            self._init_agent_deferred()
        except Exception as e:
            logger.error(f"Robot connection failed: {e}")
            state.last_error = str(e)
            self.robot = None
    
    def _init_agent_deferred(self):
        """Initialize the AI agent after robot is connected."""
        if not self.controller:
            return
            
        try:
            import os
            from core.navigation_agent import NavigationAgent
            from robots.xlerobot.tools import (
                create_move_forward, create_move_backward,
                create_turn_left, create_turn_right,
                create_slide_left, create_slide_right,
                create_end_task, create_enable_precision_mode, create_disable_precision_mode,
                create_save_note, create_enable_approach_mode, create_disable_approach_mode,
                create_speak, create_run_robot_policy, create_find_gap, create_align_to_gap
            )
            
            tools = [
                create_move_forward(self.controller),
                create_move_backward(self.controller),
                create_turn_left(self.controller),
                create_turn_right(self.controller),
                create_slide_left(self.controller),
                create_slide_right(self.controller),
                create_end_task(),
                create_enable_precision_mode(),
                create_disable_precision_mode(),
                create_save_note(),
                create_enable_approach_mode(),
                create_disable_approach_mode(),
                create_speak(),
                create_run_robot_policy(),
                create_find_gap(),
                create_align_to_gap()
            ]
            
            model_name = os.getenv("AI_MODEL", "openai/gpt-5.2")
            agent = NavigationAgent(self, model_name, tools)
            state.agent = agent
            logger.info("AI Agent ready")
        except Exception as e:
            logger.warning(f"AI Agent init failed: {e}")

    # Backwards compatibility: expose controller from robot
    @property
    def controller(self):
        if self.robot and hasattr(self.robot, 'controller'):
            return self.robot.controller
        return None

    def get_frame(self):
        if hasattr(state, 'latest_frame') and state.latest_frame is not None:
             return state.latest_frame
             
        if not self.camera:
            return None
        with self.camera_lock:
            try:
                ret, frame = self.camera.read()
                if ret:
                    return frame
            except Exception:
                pass
            return None

    def get_right_frame(self):
        """Get the latest frame from the right camera."""
        if hasattr(state, 'latest_frame_right') and state.latest_frame_right is not None:
             return state.latest_frame_right
        return None

    def cleanup(self):
        """Release resources."""
        self.running = False
        if self.robot:
            try:
                self.robot.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting robot: {e}")
        
        # Lazy import to avoid circular dependency
        from camera import release_camera
        release_camera()
            
    def emergency_stop(self):
        """Immediate stop of all movement."""
        if self.robot:
            try:
                self.robot.stop_wheels()
            except Exception:
                pass
