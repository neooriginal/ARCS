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
        self.origin_position = None
        self.origin_quaternion = None
    
    def reset_grip(self):
        self.grip_active = False
        self.origin_position = None
        self.origin_quaternion = None


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
                self._send_goal(ControlGoal(mode=ControlMode.POSITION_CONTROL))
                logger.info("VR grip activated")
            
            if ctrl.origin_position:
                delta = compute_relative_position(position, ctrl.origin_position, scale)
                
                roll, flex = 0.0, 0.0
                if ctrl.origin_quaternion is not None and quaternion and R:
                    cur_q = np.array([
                        quaternion.get('x', 0), quaternion.get('y', 0),
                        quaternion.get('z', 0), quaternion.get('w', 1)
                    ])
                    roll = self._extract_rotation(cur_q, ctrl.origin_quaternion, 2)
                    flex = self._extract_rotation(cur_q, ctrl.origin_quaternion, 0)
                
                self._send_goal(ControlGoal(
                    mode=ControlMode.POSITION_CONTROL,
                    target_position=delta,
                    wrist_roll_deg=-flex,
                    wrist_flex_deg=roll
                ))
    
    def _handle_joystick(self, stick: Dict):
        x, y = stick.get('x', 0), stick.get('y', 0)
        if abs(x) > 0.1 or abs(y) > 0.1:
            self._send_goal(ControlGoal(move_forward=-y, move_rotation=x * 0.8))
    
    def _handle_grip_release(self):
        if self.right_controller.grip_active:
            self.right_controller.reset_grip()
            self._send_goal(ControlGoal(mode=ControlMode.IDLE))
            logger.info("VR grip released")
    
    def _handle_trigger_release(self):
        if self.right_controller.trigger_active:
            self.right_controller.trigger_active = False
            self._send_goal(ControlGoal(gripper_closed=False))
    
    def _send_goal(self, goal: ControlGoal):
        if self.goal_callback:
            try:
                self.goal_callback(goal)
            except Exception as e:
                logger.error(f"Goal callback error: {e}")
    
    def _extract_rotation(self, cur: np.ndarray, origin: np.ndarray, axis: int) -> float:
        if not R:
            return 0.0
        try:
            rel = R.from_quat(cur) * R.from_quat(origin).inv()
            return -np.degrees(rel.as_rotvec()[axis])
        except:
            return 0.0
