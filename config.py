"""
RoboCrew Control System - Configuration
"""

import os

# Get the directory of this config file
_CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))

# Hardware ports
CAMERA_PORT = "/dev/video0"
WHEEL_USB = "/dev/robot_acm0"
HEAD_USB = "/dev/robot_acm1"

# Arm calibration file (in RoboCrew/calibrations)
ARM_CALIBRATION_PATH = os.path.join(_CONFIG_DIR, "RoboCrew", "calibrations", "robot_arms.json")

# Web server
WEB_PORT = 5000

# Camera settings
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
JPEG_QUALITY = 80
CAMERA_BUFFER_SIZE = 1

# Control settings
MOVEMENT_LOOP_INTERVAL = 0.05  # 50ms between movement updates
HEAD_UPDATE_INTERVAL = 33  # ~30 updates/sec for smooth control
ARM_UPDATE_INTERVAL = 50  # ~20 updates/sec for arm control
