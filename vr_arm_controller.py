"""VR Arm Controller - bridges VR input to servo control using IK."""

import time
import numpy as np
import logging
from typing import Optional, Callable

from state import state
from core.vr_kinematics import vr_kinematics, NUM_JOINTS, NUM_IK_JOINTS, WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, GRIPPER_INDEX
from core.vr_server import VRSocketHandler, ControlGoal, ControlMode

logger = logging.getLogger(__name__)


class VRArmController:
    def __init__(self, servo_controller, movement_callback: Optional[Callable] = None):
        self.servo_controller = servo_controller
        self.movement_callback = movement_callback
        
        self.mode = ControlMode.IDLE
        self.current_angles = np.zeros(NUM_JOINTS)
        self.origin_position = None
        self.origin_wrist_roll = 0.0
        self.origin_wrist_flex = 0.0
        self.gripper_closed = False
        
        self.last_movement_time = 0
        self.movement_interval = 0.05
        
        self.last_arm_update_time = 0
        self.arm_update_interval = 0.05  # 20Hz max for arm updates
        
        self.config = {'vr_scale': 1.0}
        self.vr_handler = VRSocketHandler(self._handle_goal, self.config)
        
        self._init_kinematics()
    
    def _init_kinematics(self):
        try:
            if vr_kinematics.initialize():
                logger.info("VR Kinematics initialized")
                self._sync_from_robot()
            else:
                logger.error("VR Kinematics init failed")
        except Exception as e:
            logger.error(f"Kinematics error: {e}")
    
    def _sync_from_robot(self) -> bool:
        if not self.servo_controller or not hasattr(self.servo_controller, 'get_arm_position'):
            return False
        try:
            pos = self.servo_controller.get_arm_position()
            if pos:
                new_angles = np.array([
                    pos.get('shoulder_pan', 0),
                    pos.get('shoulder_lift', 0),
                    pos.get('elbow_flex', 0),
                    pos.get('wrist_flex', 0),
                    pos.get('wrist_roll', 0),
                    pos.get('gripper', 0)
                ])
                
                if np.all(new_angles < -170):
                    logger.error(f"Sync rejected: Suspicious servo values (near -180)")
                    return False

                self.current_angles = new_angles
                vr_kinematics.update_current_angles(self.current_angles)
                return True
            else:
                logger.warning("Synced arm failed: Empty position data")
                return False
        except Exception as e:
            logger.warning(f"Sync error: {e}")
            return False
    
    def _handle_goal(self, goal: ControlGoal):
        try:
            if goal.move_forward != 0 or goal.move_rotation != 0:
                self._handle_movement(goal)
            if goal.mode is not None:
                self._handle_mode_change(goal)
            if goal.target_position is not None and self.mode == ControlMode.POSITION_CONTROL:
                self._handle_position(goal)
            if goal.gripper_closed is not None:
                self._handle_gripper(goal.gripper_closed)
        except Exception as e:
            logger.error(f"VR goal error: {e}")
    
    def _handle_movement(self, goal: ControlGoal):
        now = time.time()
        if now - self.last_movement_time < self.movement_interval:
            return
        self.last_movement_time = now
        
        fwd = goal.move_forward
        rot = goal.move_rotation

        state.movement = {
            'forward': fwd > 0.3,
            'backward': fwd < -0.3,
            'left': rot > 0.3,
            'right': rot < -0.3,
            'slide_left': False,
            'slide_right': False
        }
        state.last_movement_activity = now
        state.last_remote_activity = now
    
    def _handle_mode_change(self, goal: ControlGoal):
        if goal.mode == ControlMode.POSITION_CONTROL and self.mode != ControlMode.POSITION_CONTROL:
            if self._sync_from_robot():
                self.origin_position = vr_kinematics.get_end_effector_position(self.current_angles)
                self.origin_wrist_roll = self.current_angles[WRIST_ROLL_INDEX]
                self.origin_wrist_flex = self.current_angles[WRIST_FLEX_INDEX]
                self.mode = ControlMode.POSITION_CONTROL
            else:
                logger.error("Failed to sync arm, blocking VR engagement")
                self.mode = ControlMode.IDLE
                
        elif goal.mode == ControlMode.IDLE:
            self.mode = ControlMode.IDLE
            self.origin_position = None
    
    def _handle_position(self, goal: ControlGoal):
        if self.origin_position is None:
            return
        
        now = time.time()
        if now - self.last_arm_update_time < self.arm_update_interval:
            return
        self.last_arm_update_time = now
        
        target = self.origin_position + goal.target_position
        ik = vr_kinematics.solve_ik(target, self.current_angles)
        
        new = self.current_angles.copy()
        new[:NUM_IK_JOINTS] = ik
        
        if goal.wrist_roll_deg is not None:
            new[WRIST_ROLL_INDEX] = self.origin_wrist_roll + goal.wrist_roll_deg
        if goal.wrist_flex_deg is not None:
            new[WRIST_FLEX_INDEX] = self.origin_wrist_flex + goal.wrist_flex_deg
        
        new = np.clip(new, -120, 120)
        new[GRIPPER_INDEX] = 2 if self.gripper_closed else 90
        
        self._send_arm(new)
        self.current_angles = new
        vr_kinematics.update_current_angles(self.current_angles)
    
    def _handle_gripper(self, closed: bool):
        self.gripper_closed = closed
        if self.servo_controller:
            try:
                self.servo_controller.set_gripper(closed)
            except Exception as e:
                logger.error(f"Gripper error: {e}")
    
    def _send_arm(self, angles: np.ndarray):
        if not self.servo_controller:
            return
        try:
            self.servo_controller.set_arm_position({
                'shoulder_pan': float(angles[0]),
                'shoulder_lift': float(angles[1]),
                'elbow_flex': float(angles[2]),
                'wrist_flex': float(angles[3]),
                'wrist_roll': float(angles[4]),
                'gripper': float(angles[5])
            })
        except Exception as e:
            logger.error(f"Arm error: {e}")
    
    def cleanup(self):
        vr_kinematics.cleanup()


vr_arm_controller: Optional[VRArmController] = None
