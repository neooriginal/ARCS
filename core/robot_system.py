import threading


import logging
from typing import Optional, Dict, Any

from state import state
from robots.xlerobot.servo_controls import ServoControler
from config import WHEEL_USB, HEAD_USB, CAMERA_PORT, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_BUFFER_SIZE

logger = logging.getLogger(__name__)

class RobotSystem:
    """
    Central management for RoboCrew hardware and state.
    Handles Servos, Camera, and shared State.
    """
    def __init__(self):
        self.controller: Optional[ServoControler] = None
        self.camera = None
        self.camera_lock = threading.Lock()
        self.running = True
        
        # Initialize hardware
        self._init_camera()
        self._init_servos()
        
        state.robot_system = self
        
    def _init_camera(self):
        try:
            # Lazy import to avoid circular dependency
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

    def _init_servos(self):
        """Initialize servo controller."""
        logger.info(f"Connecting servos ({WHEEL_USB}, {HEAD_USB})...")
        try:
            self.controller = ServoControler(
                WHEEL_USB, 
                HEAD_USB,
                enable_arm=True
            )
            state.controller = self.controller
            
            # Initial readings
            if self.controller.arm_enabled:
                state.arm_connected = True
                try:
                    pos = self.controller.get_arm_position()
                    state.update_arm_positions(pos)
                except Exception as e:
                    logger.warning(f"Could not read arm: {e}")
            
            try:
                pos = self.controller.get_head_position()
                state.head_yaw = round(pos.get(7, 0), 1)
                state.head_pitch = round(pos.get(8, 0), 1)
            except Exception as e:
                logger.warning(f"Could not read head: {e}")
                
            logger.info("Servos connected successfully")
        except Exception as e:
            logger.error(f"Servo connection failed: {e}")
            state.last_error = str(e)
            self.controller = None

    def get_frame(self):
        if hasattr(state, 'latest_frame') and state.latest_frame is not None:
             return state.latest_frame
             
        if not self.camera:
            return None
        with self.camera_lock:
            ret, frame = self.camera.read()
            if ret:
                return frame
            return None

    def cleanup(self):
        """Release resources."""
        self.running = False
        if self.controller:
            try:
                self.controller.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting servos: {e}")
        
        # Lazy import to avoid circular dependency
        from camera import release_camera
        release_camera()
            
    def emergency_stop(self):
        """Immediate stop of all movement."""
        if self.controller:
            # Stop wheels
            self.controller._wheels_stop()
            # We might want to relax arm or hold position depending on safety
            # For now, just stopping wheels is critical
