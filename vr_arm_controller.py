"""VR Arm Controller - bridges VR input to servo control using IK."""

import time
import numpy as np
import logging
from typing import Optional, Callable

from state import state
from core.vr_kinematics import vr_kinematics, NUM_JOINTS, NUM_IK_JOINTS, WRIST_FLEX_INDEX, WRIST_ROLL_INDEX, GRIPPER_INDEX
from core.vr_server import VRSocketHandler, ControlGoal, ControlMode

logger = logging.getLogger(__name__)

# Motion smoothing parameters
SMOOTHING_FACTOR = 0.35        # Blend ratio per update (0=no movement, 1=instant)
MAX_STEP_DEG = 8.0             # Maximum degrees to move per update cycle
MIN_CHANGE_DEG = 0.3           # Deadband - ignore changes smaller than this


def _smooth_step(t: float) -> float:
    """Smoothstep function for ease-in/ease-out (Hermite interpolation)."""
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


class VRArmController:
    def __init__(self, servo_controller, movement_callback: Optional[Callable] = None):
        self.servo_controller = servo_controller
        self.movement_callback = movement_callback
        
        self.mode = ControlMode.IDLE
        self.current_angles = np.zeros(NUM_JOINTS)
        self.target_angles = np.zeros(NUM_JOINTS)  # Smoothed target
        self.origin_position = None
        self.origin_wrist_roll = 0.0
        self.origin_wrist_flex = 0.0
        self.gripper_closed = False
        
        self.last_movement_time = 0
        self.movement_interval = 0.05
        
        self.last_arm_update_time = 0
        self.arm_update_interval = 0.04  # 25Hz for smoother interpolation
        
        # Track motion start for acceleration curve
        self.motion_start_time = 0
        self.is_moving = False
        
        self.config = {'vr_scale': 1.0}
        self.vr_handler = VRSocketHandler(self._handle_goal, self.config)
        
        self._init_kinematics()
    
    def _init_kinematics(self) -> None:
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
                    logger.error("Sync rejected: Suspicious servo values (near -180)")
                    return False

                self.current_angles = new_angles.copy()
                self.target_angles = new_angles.copy()
                vr_kinematics.update_current_angles(self.current_angles)
                return True
            else:
                logger.warning("Synced arm failed: Empty position data")
                return False
        except Exception as e:
            logger.warning(f"Sync error: {e}")
            return False
    
    def _handle_goal(self, goal: ControlGoal) -> None:
        try:
            if goal.move_forward != 0 or goal.move_rotation != 0:
                self._handle_movement(goal)
            if goal.head_yaw_delta != 0 or goal.head_pitch_delta != 0:
                self._handle_head(goal)
            if goal.mode is not None:
                self._handle_mode_change(goal)
            if goal.target_position is not None and self.mode == ControlMode.POSITION_CONTROL:
                self._handle_position(goal)
            if goal.gripper_closed is not None:
                self._handle_gripper(goal.gripper_closed)
        except Exception as e:
            logger.error(f"VR goal error: {e}")
    
    def _handle_movement(self, goal: ControlGoal) -> None:
        now = time.time()
        if now - self.last_movement_time < self.movement_interval:
            return
        self.last_movement_time = now
        
        fwd = goal.move_forward
        rot = goal.move_rotation

        state.update_movement({
            'forward': fwd if fwd > 0 else 0.0,
            'backward': -fwd if fwd < 0 else 0.0,
            'left': rot if rot > 0 else 0.0,
            'right': -rot if rot < 0 else 0.0,
            'slide_left': 0.0,
            'slide_right': 0.0
        })
        state.last_movement_activity = now
        state.last_remote_activity = now
    
    def _handle_mode_change(self, goal: ControlGoal) -> None:
        if goal.mode == ControlMode.POSITION_CONTROL and self.mode != ControlMode.POSITION_CONTROL:
            if self._sync_from_robot():
                self.origin_position = vr_kinematics.get_end_effector_position(self.current_angles)
                self.origin_wrist_roll = self.current_angles[WRIST_ROLL_INDEX]
                self.origin_wrist_flex = self.current_angles[WRIST_FLEX_INDEX]
                self.mode = ControlMode.POSITION_CONTROL
                self.motion_start_time = time.time()
                self.is_moving = False
            else:
                logger.error("Failed to sync arm, blocking VR engagement")
                self.mode = ControlMode.IDLE
                
        elif goal.mode == ControlMode.IDLE:
            self.mode = ControlMode.IDLE
            self.origin_position = None
            self.is_moving = False
    
    def _handle_position(self, goal: ControlGoal) -> None:
        if self.origin_position is None:
            return
        
        now = time.time()
        if now - self.last_arm_update_time < self.arm_update_interval:
            return
        self.last_arm_update_time = now
        
        # Calculate raw target from IK
        target = self.origin_position + goal.target_position
        ik = vr_kinematics.solve_ik(target, self.current_angles)
        
        raw_target = self.current_angles.copy()
        raw_target[:NUM_IK_JOINTS] = ik
        
        if goal.wrist_roll_deg is not None:
            raw_target[WRIST_ROLL_INDEX] = self.origin_wrist_roll + goal.wrist_roll_deg
        if goal.wrist_flex_deg is not None:
            raw_target[WRIST_FLEX_INDEX] = self.origin_wrist_flex + goal.wrist_flex_deg
        
        raw_target = np.clip(raw_target, -120, 120)
        raw_target[GRIPPER_INDEX] = -60 if self.gripper_closed else 80
        
        # Smooth interpolation toward target
        smoothed = self._interpolate_to_target(raw_target)
        
        # Deadband check - skip if change is negligible
        max_change = np.max(np.abs(smoothed - self.current_angles))
        if max_change < MIN_CHANGE_DEG:
            return
        
        self._send_arm(smoothed)
        self.current_angles = smoothed
        vr_kinematics.update_current_angles(self.current_angles)
    
    def _interpolate_to_target(self, raw_target: np.ndarray) -> np.ndarray:
        """Interpolate current angles toward target with ease-in/ease-out."""
        delta = raw_target - self.current_angles
        distance = np.abs(delta)
        
        # Detect motion start for acceleration curve
        if not self.is_moving and np.max(distance) > MIN_CHANGE_DEG:
            self.is_moving = True
            self.motion_start_time = time.time()
        
        # Calculate acceleration factor (ramps up over first 0.15s)
        if self.is_moving:
            elapsed = time.time() - self.motion_start_time
            accel_factor = _smooth_step(min(elapsed / 0.15, 1.0))
        else:
            accel_factor = 1.0
        
        # Apply smoothing with acceleration
        effective_smoothing = SMOOTHING_FACTOR * accel_factor
        
        # Calculate step with max velocity limit
        step = delta * effective_smoothing
        
        # Clamp step size per joint
        step = np.clip(step, -MAX_STEP_DEG, MAX_STEP_DEG)
        
        return self.current_angles + step
    
    def _handle_head(self, goal: ControlGoal) -> None:
        if not self.servo_controller:
            return
        try:
            current_yaw = state.head_yaw
            current_pitch = state.head_pitch
            
            new_yaw = current_yaw + goal.head_yaw_delta
            new_pitch = current_pitch + goal.head_pitch_delta
            
            self.servo_controller.turn_head_yaw(new_yaw)
            self.servo_controller.turn_head_pitch(new_pitch)
            
            state.head_yaw = new_yaw
            state.head_pitch = new_pitch
        except Exception as e:
            logger.error(f"Head control error: {e}")
    
    def _handle_gripper(self, closed: bool) -> None:
        self.gripper_closed = closed
        if self.servo_controller:
            try:
                self.servo_controller.set_gripper(closed)
            except Exception as e:
                logger.error(f"Gripper error: {e}")
    
    def _send_arm(self, angles: np.ndarray) -> None:
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
    
    def cleanup(self) -> None:
        vr_kinematics.cleanup()


vr_arm_controller: Optional[VRArmController] = None
