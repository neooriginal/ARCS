"""
Configuration Manager
Reads/writes config.json with fallback to config.py defaults.
"""
import os
import json
import logging
from pathlib import Path
from typing import Any, Dict

logger = logging.getLogger(__name__)

CONFIG_JSON_PATH = Path("config.json")

# Default values (matching config.py)
DEFAULTS = {
    # AI Config
    "OPENAI_API_KEY": "",

    # Hardware ports
    "CAMERA_PORT": "/dev/video2",
    "CAMERA_RIGHT_PORT": "/dev/video0",
    "WHEEL_USB": "/dev/robot_acm0",
    "HEAD_USB": "/dev/robot_acm1",
    
    # Robot type (see robots/ folder for available types)
    "ROBOT_TYPE": "xlerobot",
    
    # Web server
    "WEB_PORT": 5000,
    
    # Camera Capture
    "CAMERA_WIDTH": 1280,
    "CAMERA_HEIGHT": 720,
    "CAMERA_BUFFER_SIZE": 1,
    
    # Obstacle Detection
    "OBSTACLE_SOBEL_THRESHOLD": 45,
    "OBSTACLE_THRESHOLD_RATIO": 0.875,
    
    # Video Stream
    "STREAM_WIDTH": 640,
    "STREAM_HEIGHT": 360,
    "STREAM_JPEG_QUALITY": 50,
    
    # Control intervals
    "MOVEMENT_LOOP_INTERVAL": 0.05,
    "HEAD_UPDATE_INTERVAL": 33,
    "ARM_UPDATE_INTERVAL": 50,
    
    # Arm control sensitivity
    "ARM_XY_SENSITIVITY": 0.1,
    "ARM_WRIST_SENSITIVITY": 1.0,
    "ARM_SHOULDER_PAN_STEP": 2.0,
    "ARM_WRIST_FLEX_STEP": 2.0,
    
    # Safety
    "REMOTE_TIMEOUT": 0.5,
    "AI_MIN_BRIGHTNESS": 40,
    "STALL_LOAD_THRESHOLD": 600,
    "STALL_CHECK_INTERVAL": 0.5,
    
    # Text-to-Speech
    "TTS_ENABLED": True,
    "TTS_AUDIO_DEVICE": "plughw:1,0",
    "TTS_TLD": "com",
    
    # VR Control (always enabled, no toggle)
    "VR_WEBSOCKET_PORT": 8442,
    "VR_TO_ROBOT_SCALE": 1.0,
    "VR_SEND_INTERVAL": 0.05,
    
    # Lidar Sensor (TF-Luna single-point)
    "LIDAR_PORT": "",              # Serial port (e.g., /dev/ttyUSB0)
    "LIDAR_PROTOCOL": "uart",      # "uart" or "i2c"
    "LIDAR_BAUD_RATE": 115200,     # For UART mode
    "LIDAR_I2C_ADDRESS": 16,       # Default TF-Luna I2C address (0x10 = 16)
    
    # LIDAR Obstacle Detection Thresholds
    "LIDAR_STOP_DISTANCE": 30,     # cm - block forward movement
    "LIDAR_WARN_DISTANCE": 80,     # cm - show caution zone
    "LIDAR_MAX_DISPLAY": 200,      # cm - max distance for bar display
    "LIDAR_APPROACH_DISTANCE": 2,  # cm - stop distance in Approach Mode
    
    # Head Homing Position (for ACT recording and execution)
    "HEAD_HOMING_YAW": 0.0,        # Degrees (-180 to 180), 0 = centered
    "HEAD_HOMING_PITCH": 45.0,     # Degrees (-90 to 90), default VLA position
    
    # Head Homing Position (for AI navigation)
    "HEAD_NAV_HOMING_YAW": 0.0,    # Degrees (-180 to 180), 0 = centered
    "HEAD_NAV_HOMING_PITCH": 22.0, # Degrees (-90 to 90), level for obstacle detection
}


class ConfigManager:
    def __init__(self):
        self._cache: Dict[str, Any] = {}
        self._load()
    
    def _load(self):
        """Load config from JSON, merging with defaults."""
        self._cache = DEFAULTS.copy()
        
        if CONFIG_JSON_PATH.exists():
            try:
                with open(CONFIG_JSON_PATH, 'r') as f:
                    user_config = json.load(f)
                self._cache.update(user_config)
            except Exception as e:
                logger.warning(f"Failed to load config.json: {e}")
        else:
            # Auto-create with defaults
            self._save()
            logger.info("Created default config.json")
            
        # Inject API Key into Environment for LangChain/OpenAI
        api_key = self._cache.get("OPENAI_API_KEY", "")
        if api_key:
            os.environ["OPENAI_API_KEY"] = api_key
            logger.info("OPENAI_API_KEY loaded from config")
    
    def _save(self):
        """Write current config to JSON."""
        try:
            with open(CONFIG_JSON_PATH, 'w') as f:
                json.dump(self._cache, f, indent=4)
            return True
        except Exception as e:
            logger.error(f"Failed to save config.json: {e}")
            return False
    
    def get(self, key: str, default: Any = None) -> Any:
        """Get a config value."""
        return self._cache.get(key, default)
    
    def get_all(self) -> Dict[str, Any]:
        """Get all config values."""
        return self._cache.copy()
    
    def set(self, key: str, value: Any):
        """Set a single config value (does not auto-save)."""
        self._cache[key] = value
    
    def update(self, data: Dict[str, Any]) -> bool:
        """Update multiple values and save."""
        self._cache.update(data)
        return self._save()
    
    def get_defaults(self) -> Dict[str, Any]:
        """Return default values for reference."""
        return DEFAULTS.copy()


# Singleton instance
config_manager = ConfigManager()


def get_config(key: str, default: Any = None) -> Any:
    """Convenience function to get a config value."""
    return config_manager.get(key, default)


def save_config() -> bool:
    """Convenience function to force save config."""
    return config_manager._save()
