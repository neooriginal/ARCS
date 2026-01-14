import numpy as np
import threading
import time
from typing import Dict, Optional, Any
from robots.base import BaseRobot

class MockVideoCapture:
    """Mock for cv2.VideoCapture."""
    def __init__(self, *args, **kwargs):
        self.opened = True
        self.width = 640
        self.height = 480
        
    def isOpened(self):
        return self.opened
        
    def release(self):
        self.opened = False
        
    def read(self):
        if not self.opened:
            return False, None
        # Create a dummy frame (noise with static text-like feature)
        # Using a gradient or simple pattern
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # Add random noise
        noise = np.random.randint(0, 50, (self.height, self.width, 3), dtype=np.uint8)
        frame = cv2_add(frame, noise) # Simulating cv2.add
        # Draw a moving rectangle to simulate liveness
        t = int(time.time() * 10) % self.width
        for i in range(50):
            for j in range(50):
                if 0 <= t+i < self.width and 0 <= 100+j < self.height:
                    frame[100+j, t+i] = [0, 255, 0] # Green box
        return True, frame
        
    def grab(self):
        return self.opened
        
    def retrieve(self):
        return self.read()
        
    def set(self, propId, value):
        if propId == 3: # CV_CAP_PROP_FRAME_WIDTH
            self.width = int(value)
        elif propId == 4: # CV_CAP_PROP_FRAME_HEIGHT
            self.height = int(value)
        return True
        
    def get(self, propId):
        return 0

def cv2_add(img1, img2):
    # Simple addition allowing overflow wrap-around (uint8 behavior)
    res = img1.astype(int) + img2.astype(int)
    res = np.clip(res, 0, 255).astype(np.uint8)
    return res

class MockRobot(BaseRobot):
    """Mock implementation of BaseRobot."""
    def __init__(self, name="mock_robot"):
        self._name = name
        self.connected = False
        self._arm_positions = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 90.0
        }
        self.controller = self # Mock controller behavior on itself
        self._wheel_speed = 10000
        
    @property
    def name(self) -> str:
        return self._name
        
    @property
    def has_wheels(self) -> bool:
        return True
        
    @property
    def has_head(self) -> bool:
        return True
        
    @property
    def has_arm(self) -> bool:
        return True
        
    def connect(self) -> None:
        pass # Always successful
        
    def disconnect(self) -> None:
        pass
        
    # --- Wheel/Movement ---
    def drive(self, forward: float, lateral: float = 0.0, rotation: float = 0.0) -> None:
        # Just mock receiving the command
        pass
        
    def stop_wheels(self) -> None:
        pass

    def set_speed(self, speed):
        self._wheel_speed = speed
        
    def set_velocity_vector(self, forward, lateral, rotation):
        pass

    # --- Head ---
    def move_head(self, yaw: float, pitch: float) -> None:
        pass
    
    def turn_head_yaw(self, yaw):
        return {'yaw': yaw}
        
    def turn_head_pitch(self, pitch):
        return {'pitch': pitch}

    def get_head_position(self) -> Dict[str, float]:
        return {"yaw": 0.0, "pitch": 0.0}
        
    # --- Arm ---
    def set_arm_joints(self, positions: Dict[str, float]) -> Dict[str, float]:
        self._arm_positions.update(positions)
        return self._arm_positions.copy()
        
    def set_arm_position(self, positions):
        return self.set_arm_joints(positions)

    def get_arm_joints(self) -> Dict[str, float]:
        return self._arm_positions.copy()
        
    def get_arm_position(self):
        return self.get_arm_joints()

    def set_gripper(self, closed: bool) -> float:
        self._arm_positions['gripper'] = 0.0 if closed else 90.0
        return self._arm_positions['gripper']

class MockLidar:
    def __init__(self):
        self.connected = True
        self.distance = 123 # Static dummy distance
        self._running = False
        
    def connect(self):
        return True
        
    def disconnect(self):
        self.connected = False
        
    def start_reading(self, callback=None, interval=0.1):
        self._running = True
        # In a real mock we might spawn a thread, or just manually set the state
        # For simplicity, we assume the callback is called once or the getter is used.
        # But since the system polls or uses callback, we should at least invoke it once.
        if callback:
            threading.Thread(target=self._mock_loop, args=(callback,), daemon=True).start()
            
    def _mock_loop(self, callback):
        while self._running:
            callback(self.distance)
            time.sleep(0.1)
        
    def stop_reading(self):
        self._running = False
        
    def get_distance(self):
        return self.distance

def mock_init_lidar():
    """Mock replacement for core.lidar.init_lidar"""
    from state import state
    lidar = MockLidar()
    state.lidar = lidar
    state.lidar_distance = lidar.distance
    
    # Hook up the callback manually if needed, or let the mock handle it
    def on_distance(distance):
        state.lidar_distance = distance
    lidar.start_reading(callback=on_distance)
    
    return True
