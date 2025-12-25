"""Servo controller for XLeRobot wheels, head, and arm."""

from __future__ import annotations

import time
import json
from pathlib import Path
from typing import Dict, Mapping, Optional
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode


DEFAULT_BAUDRATE = 1_000_000
DEFAULT_SPEED = 10_000
LINEAR_MPS = 0.25
ANGULAR_DPS = 90.0

ACTION_MAP = {
    "up": {7: 1.0, 8: 0.0, 9: -1.0},
    "down": {7: -1.0, 8: 0.0, 9: 1.0},
    "left": {7: 1.0, 8: 1.0, 9: 1.0},
    "right": {7: -1.0, 8: -1.0, 9: -1.0},
    "slide_left": {7: 0.15, 8: -1.0, 9: 0.15},
    "slide_right": {7: -0.15, 8: 1.0, 9: -0.15},
}

HEAD_SERVO_MAP = {"yaw": 7, "pitch": 8}

ARM_SERVO_MAP = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

ARM_LIMITS = {
    "shoulder_pan": (-90, 90),
    "shoulder_lift": (-90, 90),
    "elbow_flex": (-90, 90),
    "wrist_flex": (-90, 90),
    "wrist_roll": (-150, 150),
    "gripper": (-30, 90),
}




class ServoControler:
    """Controller for wheels (7-9), head (7-8), and arm (1-6)."""

    # Default lerobot calibration directory
    LEROBOT_CALIBRATION_DIR = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "robots"

    def __init__(
        self,
        right_arm_wheel_usb: str = None,
        left_arm_head_usb: str = None,
        *,
        speed: int = DEFAULT_SPEED,
        action_map: Optional[Mapping[str, Mapping[int, int]]] = None,
        enable_arm: bool = False,
        arm_calibration_id: str = "xlerobot_arm",
    ) -> None:
        self.right_arm_wheel_usb = right_arm_wheel_usb
        self.left_arm_head_usb = left_arm_head_usb
        self.speed = speed
        self.action_map = ACTION_MAP if action_map is None else action_map
        self._wheel_ids = tuple(sorted(next(iter(self.action_map.values())).keys()))
        self._head_ids = tuple(sorted(HEAD_SERVO_MAP.values()))
        self._arm_ids = tuple(sorted(ARM_SERVO_MAP.values()))
        
        self._arm_positions = {}
        self._arm_enabled = False
        self.wheel_bus = None
        self.head_bus = None

        if right_arm_wheel_usb:
            motors = {
                7: Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                8: Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                9: Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
            }
            
            calibration = None
            arm_ready = False
            
            if enable_arm:
                # Try to load calibration from lerobot's cache directory
                # lerobot-calibrate saves to: ~/.cache/.../robots/so101_follower/{robot_id}.json
                cal_path = self.LEROBOT_CALIBRATION_DIR / "so101_follower" / f"{arm_calibration_id}.json"
                
                if cal_path.exists():
                    try:
                        with open(cal_path) as f:
                            cal_data = json.load(f)
                        
                        arm_calibration = {}
                        # lerobot calibration format: {motor_name: {id, drive_mode, homing_offset, range_min, range_max}}
                        for motor_name, cal in cal_data.items():
                            motor_id = cal.get("id") or cal.get("motor_id")
                            if motor_id is None:
                                continue
                            arm_calibration[motor_id] = MotorCalibration(
                                id=motor_id,
                                drive_mode=cal.get("drive_mode", 0),
                                homing_offset=cal.get("homing_offset", 0),
                                range_min=cal.get("range_min", 0),
                                range_max=cal.get("range_max", 4095),
                            )
                        
                        if arm_calibration:
                            for motor_id in arm_calibration:
                                motors[motor_id] = Motor(motor_id, "sts3215", MotorNormMode.DEGREES)
                            calibration = arm_calibration
                            arm_ready = True
                            print(f"[ARM] Loaded calibration from {cal_path} ({len(arm_calibration)} motors)")
                    except Exception as e:
                        print(f"[ARM] Failed to load calibration: {e}")
                else:
                    print(f"[ARM] No calibration found at {cal_path}")
                    print(f"[ARM] Run: lerobot-calibrate --robot.type=so101_follower --robot.port={right_arm_wheel_usb} --robot.id={arm_calibration_id}")
            
            self.wheel_bus = FeetechMotorsBus(
                port=right_arm_wheel_usb,
                motors=motors,
                calibration=calibration,
            )
            
            try:
                self.wheel_bus.connect()
                self.apply_wheel_modes()
                
                if arm_ready:
                    self._apply_arm_modes()
                    self._arm_enabled = True
                    try:
                        self._arm_positions = self.get_arm_position()
                    except Exception as e:
                        print(f"[ARM] Could not read position: {e}")
            except Exception as e:
                print(f"[CONTROLLER] Error initializing with Arm: {e}")
                if arm_ready:
                    print("[CONTROLLER] Retrying with ONLY wheels...")
                    # Fallback: Re-init with only wheels
                    motors = {
                        7: Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                        8: Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                        9: Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
                    }
                    self.wheel_bus = FeetechMotorsBus(
                        port=right_arm_wheel_usb,
                        motors=motors,
                        calibration=None,
                    )
                    self.wheel_bus.connect()
                    self.apply_wheel_modes()
                    print("[CONTROLLER] Wheels connected successfully (Arm disabled)")
                else:
                    raise e
        
        head_calibration = {
            7: MotorCalibration(id=7, drive_mode=0, homing_offset=0, range_min=0, range_max=4095),
            8: MotorCalibration(id=8, drive_mode=0, homing_offset=0, range_min=0, range_max=4095),
        }
        
        if left_arm_head_usb:
            try:
                self.head_bus = FeetechMotorsBus(
                    port=left_arm_head_usb,
                    motors={
                        HEAD_SERVO_MAP["yaw"]: Motor(HEAD_SERVO_MAP["yaw"], "sts3215", MotorNormMode.DEGREES),
                        HEAD_SERVO_MAP["pitch"]: Motor(HEAD_SERVO_MAP["pitch"], "sts3215", MotorNormMode.DEGREES),
                    },
                    calibration=head_calibration,
                )
                self.head_bus.connect()
                self.apply_head_modes()
                self._head_positions = self.get_head_position()
                for sid in self._head_ids:
                    self._head_positions.setdefault(sid, 2048)
            except Exception as e:
                print(f"Warning: Could not connect to head on {left_arm_head_usb}: {e}")
                self.head_bus = None
                self._head_positions = {}

    @property
    def arm_enabled(self) -> bool:
        return self._arm_enabled

    def set_speed(self, speed: int) -> None:
        """Set the global speed for wheel motors."""
        self.speed = speed
        print(f"[CONTROLLER] Speed set to {self.speed}")

    # Wheel control

    def _wheels_write(self, action: str) -> Dict[int, int]:
        from state import state
        # Enforce Approach Mode Speed Limit (10%) ONLY for AI
        effective_speed = 1000 if (state.approach_mode and state.ai_enabled) else self.speed
        
        multipliers = self.action_map[action.lower()]
        payload = {wid: int(effective_speed * factor) for wid, factor in multipliers.items()}
        self.wheel_bus.sync_write("Goal_Velocity", payload)
        return payload

    def _wheels_stop(self) -> Dict[int, int]:
        payload = {wid: 0 for wid in self._wheel_ids}
        self.wheel_bus.sync_write("Goal_Velocity", payload)
        return payload

    def _wheels_run(self, action: str, duration: float) -> Dict[int, int]:
        if duration <= 0:
            return {}
        payload = self._wheels_write(action)
        time.sleep(duration)
        self._wheels_stop()
        return payload

    def go_forward(self, meters: float) -> Dict[int, int]:
        return self._wheels_run("up", float(meters) / LINEAR_MPS)

    def go_backward(self, meters: float) -> Dict[int, int]:
        return self._wheels_run("down", float(meters) / LINEAR_MPS)

    def turn_left(self, degrees: float) -> Dict[int, int]:
        return self._wheels_run("left", float(degrees) / ANGULAR_DPS)

    def turn_right(self, degrees: float) -> Dict[int, int]:
        return self._wheels_run("right", float(degrees) / ANGULAR_DPS)

    def slide_left(self, meters: float) -> Dict[int, int]:
        return self._wheels_run("slide_left", float(meters) / LINEAR_MPS)

    def slide_right(self, meters: float) -> Dict[int, int]:
        return self._wheels_run("slide_right", float(meters) / LINEAR_MPS)

    def set_velocity_vector(self, forward: float, lateral: float, rotation: float = 0.0) -> Dict[int, int]:
        """
        Set wheel velocities based on forward, lateral, and rotation components.
        
        Args:
            forward: Forward component (-1.0 to 1.0)
            lateral: Lateral/Slide component (-1.0 to 1.0, + is Left)
            rotation: Rotation component (-1.0 to 1.0, + is Left)
        """
        up_vec = self.action_map['up']
        slide_vec = self.action_map['slide_left']
        rot_vec = self.action_map['left']
        
        from state import state
        # Enforce Approach Mode Speed Limit (10%) ONLY for AI
        effective_speed = 1000 if (state.approach_mode and state.ai_enabled) else self.speed

        payload = {}
        for wid in self._wheel_ids:
            # Calculate combined motor factor
            u_val = up_vec.get(wid, 0)
            s_val = slide_vec.get(wid, 0)
            r_val = rot_vec.get(wid, 0)
            
            combined_factor = (forward * u_val) + (lateral * s_val) + (rotation * r_val)
            
            # Scale by effective speed
            payload[wid] = int(effective_speed * combined_factor)
            
        self.wheel_bus.sync_write("Goal_Velocity", payload)
        return payload

    def apply_wheel_modes(self) -> None:
        for wid in self._wheel_ids:
            self.wheel_bus.write("Operating_Mode", wid, OperatingMode.VELOCITY.value)
        self.wheel_bus.enable_torque()

    def get_wheel_loads(self) -> Dict[int, int]:
        """Read the current load (0-1000) from wheel motors."""
        if not self.wheel_bus:
            return {}
        try:
            return self.wheel_bus.sync_read("Present_Load", list(self._wheel_ids))
        except Exception:
            return {}

    # Head control

    def apply_head_modes(self) -> None:
        if not self.head_bus:
            return
        for sid in self._head_ids:
            self.head_bus.write("Operating_Mode", sid, OperatingMode.POSITION.value)
        self.head_bus.enable_torque()

    def turn_head_yaw(self, degrees: float) -> Dict[int, float]:
        if not self.head_bus:
            return {}
        payload = {HEAD_SERVO_MAP["yaw"]: float(degrees)}
        self.head_bus.sync_write("Goal_Position", payload)
        self._head_positions.update(payload)
        return payload

    def turn_head_pitch(self, degrees: float) -> Dict[int, float]:
        if not self.head_bus:
            return {}
        payload = {HEAD_SERVO_MAP["pitch"]: float(degrees)}
        self.head_bus.sync_write("Goal_Position", payload)
        self._head_positions.update(payload)
        return payload

    def get_head_position(self) -> Dict[int, float]:
        if not self.head_bus:
            return {}
        return self.head_bus.sync_read("Present_Position", list(self._head_ids))
    
    def turn_head_to_vla_position(self, pitch_deg=45) -> str:
        self.turn_head_pitch(pitch_deg)
        self.turn_head_yaw(0)
        time.sleep(0.9)

    def reset_head_position(self) -> str:
        self.turn_head_pitch(22)
        self.turn_head_yaw(0)
        time.sleep(0.9)

    # Arm control

    def _write_with_retry(self, bus, command: str, motor_id: int, value: int, retries: int = 3) -> bool:
        """Write to a servo with retries."""
        for attempt in range(retries):
            try:
                bus.write(command, motor_id, value)
                return True
            except Exception as e:
                if attempt == retries - 1:
                    print(f"Error writing {command} to ID {motor_id}: {e}")
                    return False
                time.sleep(0.05)
        return False

    def _apply_arm_modes(self) -> None:
        # Disable torque before changing modes
        for motor_id in self._arm_ids:
            self._write_with_retry(self.wheel_bus, "Torque_Enable", motor_id, 0)
        # Set arm to position mode
        for motor_id in self._arm_ids:
            self._write_with_retry(self.wheel_bus, "Operating_Mode", motor_id, OperatingMode.POSITION.value)
        # Re-enable torque for all motors on the bus
        self.wheel_bus.enable_torque()

    def get_arm_position(self) -> Dict[str, float]:
        if not self._arm_enabled:
            return {}
        try:
            raw = self.wheel_bus.sync_read("Present_Position", list(self._arm_ids))
        except Exception:
            # Return empty on read failure to avoid crashing loop
            return {}
            
        result = {}
        for joint_name, motor_id in ARM_SERVO_MAP.items():
            result[joint_name] = raw.get(motor_id, 0.0)
        return result

    def set_arm_position(self, positions: Dict[str, float]) -> Dict[str, float]:
        if not self._arm_enabled:
            return {}
        
        payload = {}
        for joint_name, angle in positions.items():
            if joint_name in ARM_SERVO_MAP:
                motor_id = ARM_SERVO_MAP[joint_name]
                limits = ARM_LIMITS.get(joint_name, (-180, 180))
                clamped = max(limits[0], min(limits[1], float(angle)))
                payload[motor_id] = clamped
                self._arm_positions[joint_name] = clamped
        
        if payload:
            try:
                self.wheel_bus.sync_write("Goal_Position", payload)
            except Exception as e:
                print(f"Failed to write arm positions: {e}")
                
        return self._arm_positions.copy()

    def set_arm_joint(self, joint_name: str, angle: float) -> float:
        if joint_name not in ARM_SERVO_MAP:
            return 0.0
        result = self.set_arm_position({joint_name: angle})
        return result.get(joint_name, 0.0)

    def set_gripper(self, closed: bool) -> float:
        angle = -30.0 if closed else 90.0
        return self.set_arm_joint("gripper", angle)

    # Stall Detection

    def get_head_loads(self) -> Dict[int, int]:
        """Read the current load (0-1000) from head motors."""
        if not self.head_bus:
            return {}
        try:
            return self.head_bus.sync_read("Present_Load", list(self._head_ids))
        except Exception:
            return {}

    def get_arm_loads(self) -> Dict[int, int]:
        """Read the current load (0-1000) from arm motors."""
        if not self._arm_enabled:
            return {}
        try:
            # Remap from motor_id to joint_name or just return motor_id map
            return self.wheel_bus.sync_read("Present_Load", list(self._arm_ids))
        except Exception:
            return {}

    def check_stall(self, threshold: int = 600) -> Optional[str]:
        """
        Check for stalled motors (load > threshold).
        If stalled, disabling torque for safety.
        Returns description of stall or None.
        """
        warnings = []
        
        # Check Head
        head_loads = self.get_head_loads()
        for mid, load in head_loads.items():
            if abs(load) > threshold:
                warnings.append(f"Head Motor {mid} stalled (Load: {load})")
                self._write_with_retry(self.head_bus, "Torque_Enable", mid, 0)

        # Check Arm
        if self._arm_enabled:
            arm_loads = self.get_arm_loads()
            for mid, load in arm_loads.items():
                if abs(load) > threshold:
                    warnings.append(f"Arm Motor {mid} stalled (Load: {load})")
                    self._write_with_retry(self.wheel_bus, "Torque_Enable", mid, 0)
        
        if warnings:
            return "; ".join(warnings)
        return None

    # Cleanup

    def disconnect(self) -> None:
        self._wheels_stop()
        if self.wheel_bus:
            self.wheel_bus.disconnect()
        if self.head_bus:
            self.head_bus.disconnect()

    def __del__(self) -> None:
        if hasattr(self, "wheel_bus") and self.wheel_bus and self.wheel_bus.is_connected:
            self.disconnect()

