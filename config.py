"""ARCS Configuration"""

# Hardware ports
CAMERA_PORT = "/dev/video0"
WHEEL_USB = "/dev/robot_acm0"
HEAD_USB = "/dev/robot_acm1"

# Web server
WEB_PORT = 5000

# Camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_BUFFER_SIZE = 1
JPEG_QUALITY = 50

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
