"""
RoboCrew Control System - Arm Control Module
Handles arm movement with simplified IK-like X/Y control.
"""

import math
from state import state
from config import (
    ARM_XY_SENSITIVITY,
    ARM_WRIST_SENSITIVITY,
    ARM_SHOULDER_PAN_STEP,
    ARM_WRIST_FLEX_STEP,
)


class ArmController:
    """
    Simplified arm controller that maps mouse X/Y to arm movement.
    
    The arm uses a simplified model:
    - Mouse Y (vertical) → shoulder_lift + elbow_flex (reach forward/back)
    - Mouse X (horizontal) → shoulder_pan (rotate base)
    - Mouse wheel → wrist_roll
    - Keyboard Q/E → shoulder_pan (coarse adjustment)
    - Keyboard R/F → wrist_flex
    - Mouse click → gripper
    """
    
    def __init__(self):
        # Current target positions
        self.targets = {
            'shoulder_pan': 0,
            'shoulder_lift': 0,
            'elbow_flex': 0,
            'wrist_flex': 0,
            'wrist_roll': 0,
            'gripper': 90  # Open
        }
        
        # Virtual X/Y position for IK-like mapping
        self.virtual_x = 0.0  # -1 to 1, maps to shoulder_pan
        self.virtual_y = 0.0  # -1 to 1, maps to reach (shoulder_lift + elbow_flex)
    
    def reset_to_home(self):
        """Reset arm to home position."""
        self.targets = {
            'shoulder_pan': 0,
            'shoulder_lift': 0,
            'elbow_flex': 0,
            'wrist_flex': 0,
            'wrist_roll': 0,
            'gripper': 90
        }
        self.virtual_x = 0.0
        self.virtual_y = 0.0
        return self.targets.copy()
    
    def handle_mouse_move(self, delta_x, delta_y):
        """
        Handle mouse movement to control arm position.
        
        Args:
            delta_x: Horizontal mouse movement (pixels)
            delta_y: Vertical mouse movement (pixels)
        
        Returns:
            Updated target positions
        """
        # Update virtual position
        self.virtual_x += delta_x * ARM_XY_SENSITIVITY * 0.01
        self.virtual_y += delta_y * ARM_XY_SENSITIVITY * 0.01
        
        # Clamp virtual position
        self.virtual_x = max(-1.0, min(1.0, self.virtual_x))
        self.virtual_y = max(-1.0, min(1.0, self.virtual_y))
        
        # Map virtual_x to shoulder_pan (-90 to 90 degrees)
        self.targets['shoulder_pan'] = self.virtual_x * 90
        
        # Map virtual_y to reach (shoulder_lift + elbow_flex)
        # Positive Y (mouse down) = extend arm forward
        # This is a simplified "IK" - we coordinate the two joints
        reach = self.virtual_y * 45  # -45 to 45 degrees
        self.targets['shoulder_lift'] = reach
        self.targets['elbow_flex'] = -reach * 0.5  # Counter-rotate elbow
        
        # Keep wrist level by compensating
        self.targets['wrist_flex'] = -(self.targets['shoulder_lift'] + self.targets['elbow_flex'])
        
        return self.targets.copy()
    
    def handle_scroll(self, delta):
        """
        Handle mouse scroll to control wrist roll.
        
        Args:
            delta: Scroll delta (positive = scroll up)
        
        Returns:
            Updated wrist_roll value
        """
        self.targets['wrist_roll'] += delta * ARM_WRIST_SENSITIVITY
        self.targets['wrist_roll'] = max(-150, min(150, self.targets['wrist_roll']))
        return self.targets['wrist_roll']
    
    def handle_shoulder_pan(self, direction):
        """
        Handle keyboard Q/E for shoulder pan.
        
        Args:
            direction: 1 for right (E), -1 for left (Q)
        """
        self.targets['shoulder_pan'] += direction * ARM_SHOULDER_PAN_STEP
        self.targets['shoulder_pan'] = max(-90, min(90, self.targets['shoulder_pan']))
        self.virtual_x = self.targets['shoulder_pan'] / 90.0
        return self.targets['shoulder_pan']
    
    def handle_wrist_flex(self, direction):
        """
        Handle keyboard R/F for wrist flex.
        
        Args:
            direction: 1 for up (R), -1 for down (F)
        """
        self.targets['wrist_flex'] += direction * ARM_WRIST_FLEX_STEP
        self.targets['wrist_flex'] = max(-90, min(90, self.targets['wrist_flex']))
        return self.targets['wrist_flex']
    
    def set_gripper(self, closed):
        """
        Set gripper state.
        
        Args:
            closed: True to close gripper, False to open
        """
        self.targets['gripper'] = 2 if closed else 90  # 2 = closed, 90 = open
        return self.targets['gripper']
    
    def get_targets(self):
        """Get current target positions."""
        return self.targets.copy()
    
    def set_from_current(self, positions):
        """
        Initialize targets from current arm positions.
        
        Args:
            positions: Dict of joint positions from robot
        """
        for joint, angle in positions.items():
            if joint in self.targets:
                self.targets[joint] = angle
        
        # Update virtual position based on actual arm state
        self.virtual_x = self.targets['shoulder_pan'] / 90.0
        self.virtual_y = self.targets['shoulder_lift'] / 45.0


# Global arm controller instance
arm_controller = ArmController()
