"""
RoboCrew Control System - Robot State Management
Thread-safe state container for robot hardware.
"""

import threading


class RobotState:
    """Thread-safe global state for robot hardware."""
    
    def __init__(self):
        self.camera = None
        self.camera_right = None
        self.controller = None
        self.latest_frame = None       # Threaded capture frame buffer
        self.latest_frame_right = None # Right camera buffer
        self.frame_id = 0              # Synchronization counter
        self.frame_id_right = 0        # Right camera counter
        self.running = True
        self.movement = {
            'forward': False,
            'backward': False,
            'left': False,
            'right': False,
            'slide_left': False,
            'slide_right': False
        }
        self.lock = threading.Lock()
        self.last_error = None
        
        # Head position
        self.head_yaw = 0
        self.head_pitch = 0
        
        # Control mode: 'drive' or 'arm'
        self.control_mode = 'drive'
        
        # Arm state
        self.arm_connected = False
        self.arm_positions = {
            'shoulder_pan': 0,
            'shoulder_lift': 0,
            'elbow_flex': 0,
            'wrist_flex': 0,
            'wrist_roll': 0,
            'gripper': 90  # Open by default
        }
        self.gripper_closed = False
        
        # Precision Mode (for alignment visualization)
        self.precision_mode = False
        
        # Approach Mode (for close interaction)
        self.approach_mode = False
        
        # AI State
        self.ai_enabled = False
        self.ai_status = "Idle"
        self.ai_logs = []
        self.robot_system = None
        self.agent = None
        
        # Remote control tracking
        self.last_remote_activity = 0   # Last input timestamp
        self.last_movement_activity = 0 # Last movement command timestamp
        
        # Wheel speed control
        self.default_wheel_speed = 10000 
        self.manual_wheel_speed = None  # None = use default
        self.safety_warning_triggered = False
        
        # Shared Obstacle Detector
        self.detector = None
        
        # VINS-SLAM
        # Removed per user request

    
    def update_movement(self, data):
        """Update movement state from request data."""
        with self.lock:
            self.movement = {
                'forward': bool(data.get('forward')),
                'backward': bool(data.get('backward')),
                'left': bool(data.get('left')),
                'right': bool(data.get('right')),
                'slide_left': bool(data.get('slide_left')),
                'slide_right': bool(data.get('slide_right'))
            }
    
    def get_movement(self):
        """Get a copy of current movement state."""
        with self.lock:
            return self.movement.copy()
    
    def stop_all_movement(self):
        """Stop all movement."""
        with self.lock:
            self.movement = {
                'forward': False,
                'backward': False,
                'left': False,
                'right': False,
                'slide_left': False,
                'slide_right': False
            }
    
    def set_control_mode(self, mode):
        """Set control mode ('drive' or 'arm')."""
        with self.lock:
            if mode in ('drive', 'arm'):
                self.control_mode = mode
                return True
            return False
    
    def get_control_mode(self):
        """Get current control mode."""
        with self.lock:
            return self.control_mode
    
    def update_arm_positions(self, positions):
        """Update arm position state."""
        with self.lock:
            for joint, angle in positions.items():
                if joint in self.arm_positions:
                    self.arm_positions[joint] = angle
    
    def get_arm_positions(self):
        """Get a copy of current arm positions."""
        with self.lock:
            return self.arm_positions.copy()

    def get_detector(self):
        """Get or create shared ObstacleDetector instance."""
        with self.lock:
            if self.detector is None:
                try:
                    # Lazy import to avoid circular dependency
                    import sys, os
                    if os.getcwd() not in sys.path:
                        sys.path.append(os.getcwd())
                    from obstacle_detection import ObstacleDetector
                    self.detector = ObstacleDetector()
                except Exception as e:
                    print(f"Error creating detector: {e}")
                    return None
            return self.detector
    
    def set_wheel_speed(self, speed):
        """Set manual wheel speed."""
        with self.lock:
            # Clamp speed between 1000 and 20000
            self.manual_wheel_speed = max(1000, min(20000, int(speed)))
            
            # Trigger safety warning if exceeding 13000
            if self.manual_wheel_speed > 13000 and not self.safety_warning_triggered:
                self.safety_warning_triggered = True
                # Lazy import to avoid circular dependency
                import tts
                tts.speak("Safety limiters off")
            elif self.manual_wheel_speed <= 13000 and self.safety_warning_triggered:
                # Reset warning flag when back in safe range
                self.safety_warning_triggered = False
            
            if self.controller and hasattr(self.controller, 'set_speed'):
                self.controller.set_speed(self.manual_wheel_speed)
    
    def get_wheel_speed(self):
        """Get current wheel speed."""
        with self.lock:
            if self.manual_wheel_speed is not None:
                return self.manual_wheel_speed
            return self.default_wheel_speed
    
    def reset_wheel_speed(self):
        """Reset wheel speed to default."""
        with self.lock:
            self.manual_wheel_speed = None
            self.safety_warning_triggered = False  # Reset warning flag
            if self.controller and hasattr(self.controller, 'set_speed'):
                self.controller.set_speed(self.default_wheel_speed)

    def add_ai_log(self, message: str):
        """Add a log message to AI logs."""
        with self.lock:
            import datetime
            timestamp = datetime.datetime.now().strftime("%H:%M:%S")
            self.ai_logs.append(f"[{timestamp}] {message}")
            # Keep last 100 logs
            if len(self.ai_logs) > 100:
                self.ai_logs.pop(0)
            self.ai_status = message  # Update status to latest log



# Global state instance
state = RobotState()
