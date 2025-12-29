"""ARCS Configuration"""

# Hardware ports
CAMERA_PORT = "/dev/video0"
WHEEL_USB = "/dev/robot_acm0"
HEAD_USB = "/dev/robot_acm1"

# Web server
WEB_PORT = 5000

# Camera Capture (AI/Robot Vision)
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_BUFFER_SIZE = 1

# Video Stream (Web/VR)
STREAM_WIDTH = 640
STREAM_HEIGHT = 360
STREAM_JPEG_QUALITY = 20

# Control intervals (ms)
MOVEMENT_LOOP_INTERVAL = 0.05
HEAD_UPDATE_INTERVAL = 33
ARM_UPDATE_INTERVAL = 50

# Arm control sensitivity
ARM_XY_SENSITIVITY = 0.1
ARM_WRIST_SENSITIVITY = 1.0
ARM_SHOULDER_PAN_STEP = 2.0
ARM_WRIST_FLEX_STEP = 2.0

# Safety
REMOTE_TIMEOUT = 0.5
AI_MIN_BRIGHTNESS = 40
STALL_LOAD_THRESHOLD = 600
STALL_CHECK_INTERVAL = 0.5

# Text-to-Speech
TTS_ENABLED = True
TTS_AUDIO_DEVICE = "plughw:1,0"  # ALSA device (plughw:1,0 = HDMI 1)
TTS_TLD = "com"  # Google TLD for voice variant (com, co.uk, com.au, etc.)

# VR Control
VR_ENABLED = True
VR_WEBSOCKET_PORT = 8442
VR_TO_ROBOT_SCALE = 1.0
VR_SEND_INTERVAL = 0.05

