"""
RoboCrew Control System - Configuration
"""

# Hardware ports
CAMERA_PORT = "/dev/video0"
WHEEL_USB = "/dev/robot_acm0"
HEAD_USB = "/dev/robot_acm1"

# Web server
WEB_PORT = 5000

# Camera settings
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_BUFFER_SIZE = 1
JPEG_QUALITY = 50

# Control settings
MOVEMENT_LOOP_INTERVAL = 0.05  # 50ms between movement updates
HEAD_UPDATE_INTERVAL = 33  # ~30 updates/sec for smooth control
