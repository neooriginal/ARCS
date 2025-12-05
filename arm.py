"""RoboCrew Arm Control Module"""

from state import state
from config import (
    ARM_XY_SENSITIVITY,
    ARM_WRIST_SENSITIVITY,
    ARM_SHOULDER_PAN_STEP,
    ARM_WRIST_FLEX_STEP,
)

ARM_ELBOW_STEP = 2.0


class ArmController:
    """Direct joint control for robot arm."""
    
    def __init__(self):
        self.targets = {
            'shoulder_pan': 0,
            'shoulder_lift': 0,
            'elbow_flex': 0,
            'wrist_flex': 0,
            'wrist_roll': 0,
            'gripper': 90
        }
    
    def reset_to_home(self):
        self.targets = {
            'shoulder_pan': 0,
            'shoulder_lift': 0,
            'elbow_flex': 0,
            'wrist_flex': 0,
            'wrist_roll': 0,
            'gripper': 90
        }
        return self.targets.copy()
    
    def handle_mouse_move(self, delta_x, delta_y):
        self.targets['shoulder_pan'] += delta_x * ARM_XY_SENSITIVITY
        self.targets['shoulder_pan'] = max(-90, min(90, self.targets['shoulder_pan']))
        
        self.targets['shoulder_lift'] += delta_y * ARM_XY_SENSITIVITY
        self.targets['shoulder_lift'] = max(-90, min(90, self.targets['shoulder_lift']))
        
        return self.targets.copy()
    
    def handle_scroll(self, delta):
        self.targets['wrist_roll'] += delta * ARM_WRIST_SENSITIVITY
        self.targets['wrist_roll'] = max(-150, min(150, self.targets['wrist_roll']))
        return self.targets['wrist_roll']
    
    def handle_shoulder_pan(self, direction):
        self.targets['shoulder_pan'] += direction * ARM_SHOULDER_PAN_STEP
        self.targets['shoulder_pan'] = max(-90, min(90, self.targets['shoulder_pan']))
        return self.targets['shoulder_pan']
    
    def handle_wrist_flex(self, direction):
        self.targets['wrist_flex'] += direction * ARM_WRIST_FLEX_STEP
        self.targets['wrist_flex'] = max(-90, min(90, self.targets['wrist_flex']))
        return self.targets['wrist_flex']
    
    def handle_elbow_flex(self, direction):
        self.targets['elbow_flex'] += direction * ARM_ELBOW_STEP
        self.targets['elbow_flex'] = max(-90, min(90, self.targets['elbow_flex']))
        return self.targets['elbow_flex']
    
    def set_gripper(self, closed):
        self.targets['gripper'] = 2 if closed else 90
        return self.targets['gripper']
    
    def get_targets(self):
        return self.targets.copy()
    
    def set_from_current(self, positions):
        for joint, angle in positions.items():
            if joint in self.targets:
                self.targets[joint] = angle


arm_controller = ArmController()
