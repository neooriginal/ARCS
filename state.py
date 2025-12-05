"""
RoboCrew Control System - Robot State Management
Thread-safe state container for robot hardware.
"""

import threading


class RobotState:
    """Thread-safe global state for robot hardware."""
    
    def __init__(self):
        self.camera = None
        self.controller = None
        self.running = True
        self.movement = {
            'forward': False,
            'backward': False,
            'left': False,
            'right': False
        }
        self.lock = threading.Lock()
        self.last_error = None
        # Current head position - read from servos at startup
        self.head_yaw = 0
        self.head_pitch = 0
    
    def update_movement(self, data):
        """Update movement state from request data."""
        with self.lock:
            self.movement = {
                'forward': bool(data.get('forward')),
                'backward': bool(data.get('backward')),
                'left': bool(data.get('left')),
                'right': bool(data.get('right'))
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
                'right': False
            }


# Global state instance
state = RobotState()
