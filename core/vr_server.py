"""VR Socket Handler for RoboCrew - processes Quest 3 controller data via Socket.IO."""

import numpy as np
import logging
from typing import Optional, Dict
from dataclasses import dataclass, field
from enum import Enum

try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    R = None

from .vr_kinematics import compute_relative_position

logger = logging.getLogger(__name__)


class ControlMode(Enum):
    IDLE = "idle"
    POSITION_CONTROL = "position_control"


@dataclass
class ControlGoal:
    mode: Optional[ControlMode] = None
    target_position: Optional[np.ndarray] = None
    wrist_roll_deg: Optional[float] = None
    wrist_flex_deg: Optional[float] = None
    gripper_closed: Optional[bool] = None
    move_forward: float = 0.0
    move_lateral: float = 0.0
    move_rotation: float = 0.0
    metadata: Dict = field(default_factory=dict)


class VRControllerState:
    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        self.trigger_active = False
        
        # Position tracking relative movement
        self.origin_position = None
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None
        
        # Rotation tracking
        self.z_axis_rotation = 0.0
        self.x_axis_rotation = 0.0
    
    def reset_grip(self):
        self.grip_active = False
        self.origin_position = None
        self.origin_quaternion = None
        self.accumulated_rotation_quat = None
        self.z_axis_rotation = 0.0
        self.x_axis_rotation = 0.0


class VRSocketHandler:
    def __init__(self, goal_callback, config):
        self.goal_callback = goal_callback
        self.config = config
        self.right_controller = VRControllerState("right")
        self.connected_clients = 0
        self.is_running = False
    
    def on_connect(self):
        self.connected_clients += 1
        self.is_running = True
        logger.info(f"VR client connected ({self.connected_clients})")
    
    def on_disconnect(self):
        self.connected_clients = max(0, self.connected_clients - 1)
        self.is_running = self.connected_clients > 0
        
        if self.right_controller.grip_active:
            self.right_controller.reset_grip()
            self._send_goal(ControlGoal(mode=ControlMode.IDLE))
        logger.info(f"VR client disconnected ({self.connected_clients})")
    
    def on_vr_data(self, data: Dict):
        try:
            if 'rightController' in data:
                # DEBUG: Trace data receipt
                # logger.info(f"VR Data: {data.keys()}") 
                right = data['rightController']
                if right.get('position'):
                    self._process_controller(right)
                elif not right.get('gripActive') and self.right_controller.grip_active:
                    self._handle_grip_release()
                
                if 'thumbstick' in right:
                    self._handle_joystick(right['thumbstick'])
                
                left = data.get('leftController', {})
                if 'thumbstick' in left:
                    self._handle_joystick(left['thumbstick'])
                return
            
            if data.get('gripReleased'):
                self._handle_grip_release()
            elif data.get('triggerReleased'):
                self._handle_trigger_release()
            elif data.get('position'):
                self._process_controller(data)
        except Exception as e:
            logger.error(f"VR data error: {e}")
    
    def _process_controller(self, data: Dict):
        position = data.get('position', {})
        quaternion = data.get('quaternion', {})
        grip_active = data.get('gripActive', False)
        trigger = data.get('trigger', 0)
        
        ctrl = self.right_controller
        scale = self.config.get('vr_scale', 1.0)
        
        trigger_active = trigger > 0.5
        if trigger_active != ctrl.trigger_active:
            ctrl.trigger_active = trigger_active
            self._send_goal(ControlGoal(gripper_closed=trigger_active))
        
        if grip_active:
            if not ctrl.grip_active:
                ctrl.grip_active = True
                ctrl.origin_position = position.copy()
                
                if quaternion:
                    ctrl.origin_quaternion = np.array([
                        quaternion.get('x', 0), quaternion.get('y', 0),
                        quaternion.get('z', 0), quaternion.get('w', 1)
                    ])
                    ctrl.accumulated_rotation_quat = ctrl.origin_quaternion
                
                self._send_goal(ControlGoal(mode=ControlMode.POSITION_CONTROL))
            
            if ctrl.origin_position:
                delta = compute_relative_position(position, ctrl.origin_position, scale)
                
                if quaternion and R:
                    current_quat = np.array([
                        quaternion.get('x', 0), quaternion.get('y', 0),
                        quaternion.get('z', 0), quaternion.get('w', 1)
                    ])
                    
                    if ctrl.origin_quaternion is not None:
                        ctrl.accumulated_rotation_quat = current_quat
                        ctrl.z_axis_rotation = self._extract_roll_from_quaternion(current_quat, ctrl.origin_quaternion)
                        ctrl.x_axis_rotation = self._extract_pitch_from_quaternion(current_quat, ctrl.origin_quaternion)

                self._send_goal(ControlGoal(
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=delta,
                    wrist_roll_deg=-ctrl.z_axis_rotation,
                    wrist_flex_deg=-ctrl.x_axis_rotation
                ))
    
    def _handle_joystick(self, stick: Dict):
        x, y = stick.get('x', 0), stick.get('y', 0)
        
        # Precision Mode: Reduce speed if grip is active (arm control mode)
        # This allows precise positioning while manipulating the arm
        scale = 0.3 if self.right_controller.grip_active else 1.0
        
        # Send goal if outside deadzone, OR if we need to send a final stop command (0,0)
        if abs(x) > 0.1 or abs(y) > 0.1:
            # Invert x (rotation) to fix direction, and use y for forward/back
            # Apply precision scale
            self._send_goal(ControlGoal(
                move_forward=-y * scale, 
                move_rotation=-x * 0.8 * scale
            ))
        else:
            # Send stop command when stick is centered to prevent "moving too much"
            self._send_goal(ControlGoal(move_forward=0.0, move_rotation=0.0))
    
    def _handle_grip_release(self):
        if self.right_controller.grip_active:
            self.right_controller.reset_grip()
            self._send_goal(ControlGoal(mode=ControlMode.IDLE))
    
    def _handle_trigger_release(self):
        self.right_controller.trigger_active = False
        self._send_goal(ControlGoal(gripper_closed=False))
    
    def _send_goal(self, goal: ControlGoal):
        if self.goal_callback:
            try:
                self.goal_callback(goal)
            except Exception as e:
                logger.error(f"Goal callback error: {e}")

    def _extract_roll_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray) -> float:
        """Extract roll (around Z) from relative rotation."""
        if current_quat is None or origin_quat is None or not R:
            return 0.0
        try:
            # Relative rotation: origin -> current
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()
            
            # Z-component of rotation vector is roll around Z
            rotvec = relative_rotation.as_rotvec()
            return -np.degrees(rotvec[2])
        except Exception as e:
            logger.warning(f"Error extracting roll: {e}")
            return 0.0

    def _extract_pitch_from_quaternion(self, current_quat: np.ndarray, origin_quat: np.ndarray) -> float:
        """Extract pitch (around X) from relative rotation."""
        if current_quat is None or origin_quat is None or not R:
            return 0.0
        try:
            # Same relative rotation
            origin_rotation = R.from_quat(origin_quat)
            current_rotation = R.from_quat(current_quat)
            relative_rotation = current_rotation * origin_rotation.inv()
            
            # X-component is pitch around X
            rotvec = relative_rotation.as_rotvec()
            return np.degrees(rotvec[0])
        except Exception as e:
            logger.warning(f"Error extracting pitch: {e}")
            return 0.0
