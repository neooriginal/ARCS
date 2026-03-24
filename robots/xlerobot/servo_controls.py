"""Servo controller for XLeRobot wheels and arm."""

from __future__ import annotations

import time
import json
import threading
from contextlib import contextmanager
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode


DEFAULT_BAUDRATE = 1_000_000
DEFAULT_SPEED = 10_000
LINEAR_MPS = 0.25
ANGULAR_DPS = 90.0


# 4-wheel square omni layout: 7=FL, 8=FR, 9=RL, 10=RR
# Signs assume standard mecanum mounting (motors diagonal to each other mirror-mounted)
ACTION_MAP = {
    "up":          {7:  1.0, 8: -1.0, 9:  1.0, 10: -1.0},
    "down":        {7: -1.0, 8:  1.0, 9: -1.0, 10:  1.0},
    "left":        {7:  1.0, 8:  1.0, 9:  1.0, 10:  1.0},
    "right":       {7: -1.0, 8: -1.0, 9: -1.0, 10: -1.0},
    "slide_left":  {7: -1.0, 8:  1.0, 9:  1.0, 10: -1.0},
    "slide_right": {7:  1.0, 8: -1.0, 9: -1.0, 10:  1.0},
}


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
    "gripper": (-65, 95),
}

ARM_JOINT_ORDER = tuple(ARM_SERVO_MAP.keys())
WHEEL_SERVO_IDS = tuple(sorted(next(iter(ACTION_MAP.values())).keys()))
SERVO_MODEL = "sts3215"


def _make_probe_motors(motor_ids: Iterable[int]) -> Dict[int, Motor]:
    return {
        int(motor_id): Motor(int(motor_id), SERVO_MODEL, MotorNormMode.RANGE_M100_100)
        for motor_id in sorted({int(motor_id) for motor_id in motor_ids})
    }


def _wrapped_position_delta(start: int, end: int, modulus: int = 4096) -> int:
    delta = abs(int(end) - int(start)) % modulus
    return min(delta, modulus - delta)


@contextmanager
def _calibration_bus(
    port: str,
    motor_ids: Iterable[int],
    baudrate: int = DEFAULT_BAUDRATE,
    controller: Optional["ServoControler"] = None,
):
    motor_ids = sorted({int(motor_id) for motor_id in motor_ids})
    if not motor_ids:
        raise ValueError("At least one motor ID is required")

    if (
        controller
        and controller.right_arm_wheel_usb == port
        and controller.wheel_bus is not None
        and controller.wheel_bus.is_connected
    ):
        bus = controller.wheel_bus
        with controller._bus_lock:
            original_motors = dict(bus.motors)
            original_baudrate = None
            try:
                original_baudrate = bus.get_baudrate()
            except Exception:
                pass
            bus.motors = _make_probe_motors(set(original_motors) | set(motor_ids))
            try:
                if baudrate and original_baudrate != baudrate:
                    bus.set_baudrate(baudrate)
                yield bus
            finally:
                if original_baudrate and original_baudrate != baudrate:
                    try:
                        bus.set_baudrate(original_baudrate)
                    except Exception:
                        pass
                bus.motors = original_motors
        return

    bus = FeetechMotorsBus(port=port, motors=_make_probe_motors(motor_ids), calibration=None)
    bus.connect()
    try:
        if baudrate:
            bus.set_baudrate(baudrate)
        yield bus
    finally:
        bus.disconnect()


def scan_servo_bus(
    port: str,
    *,
    controller: Optional["ServoControler"] = None,
) -> Dict[str, object]:
    if not port:
        raise ValueError("Servo port is required")

    if (
        controller
        and controller.right_arm_wheel_usb == port
        and controller.wheel_bus is not None
        and controller.wheel_bus.is_connected
    ):
        with controller._bus_lock:
            ids_models = controller.wheel_bus.broadcast_ping() or {}
            baudrate = controller.wheel_bus.get_baudrate()
        baudrate_ids = {int(baudrate): sorted(int(motor_id) for motor_id in ids_models)}
    else:
        raw_scan = FeetechMotorsBus.scan_port(port)
        baudrate_ids = {
            int(baudrate): sorted(int(motor_id) for motor_id in motor_ids)
            for baudrate, motor_ids in raw_scan.items()
        }

    if not baudrate_ids:
        return {
            "baudrate_map": {},
            "recommended_baudrate": DEFAULT_BAUDRATE,
            "all_ids": [],
            "arm_candidate_ids": [],
        }

    recommended_baudrate = DEFAULT_BAUDRATE
    if recommended_baudrate not in baudrate_ids:
        recommended_baudrate = max(baudrate_ids.items(), key=lambda item: len(item[1]))[0]

    all_ids = sorted({motor_id for ids in baudrate_ids.values() for motor_id in ids})
    return {
        "baudrate_map": baudrate_ids,
        "recommended_baudrate": int(recommended_baudrate),
        "all_ids": all_ids,
        "arm_candidate_ids": [motor_id for motor_id in all_ids if motor_id not in WHEEL_SERVO_IDS],
    }


def identify_servo_by_movement(
    port: str,
    candidate_ids: Iterable[int],
    *,
    baudrate: int = DEFAULT_BAUDRATE,
    controller: Optional["ServoControler"] = None,
    sample_seconds: float = 3.0,
    settle_seconds: float = 0.5,
    poll_interval: float = 0.08,
    movement_threshold: int = 40,
) -> Dict[str, object]:
    candidate_ids = sorted({int(motor_id) for motor_id in candidate_ids})
    if not candidate_ids:
        raise ValueError("No candidate IDs provided")

    with _calibration_bus(port, candidate_ids, baudrate=baudrate, controller=controller) as bus:
        bus.disable_torque(candidate_ids)
        try:
            time.sleep(max(0.0, settle_seconds))
            baseline = bus.sync_read("Present_Position", candidate_ids, normalize=False)
            movement_scores = {motor_id: 0 for motor_id in candidate_ids}

            started_at = time.time()
            while time.time() - started_at < max(0.5, sample_seconds):
                positions = bus.sync_read("Present_Position", candidate_ids, normalize=False)
                for motor_id in candidate_ids:
                    start = baseline.get(motor_id)
                    current = positions.get(motor_id)
                    if start is None or current is None:
                        continue
                    movement_scores[motor_id] = max(
                        movement_scores[motor_id],
                        _wrapped_position_delta(start, current),
                    )
                time.sleep(max(0.02, poll_interval))
        finally:
            try:
                bus.enable_torque(candidate_ids)
            except Exception:
                pass

    detected_id, detected_score = max(movement_scores.items(), key=lambda item: item[1])
    if detected_score < movement_threshold:
        raise RuntimeError(
            f"No motor moved enough to identify confidently. Highest movement was ID {detected_id} at {detected_score} ticks."
        )

    return {
        "detected_id": int(detected_id),
        "movement_scores": movement_scores,
        "movement_threshold": int(movement_threshold),
    }


def assign_arm_servo_ids(
    port: str,
    joint_to_current_id: Mapping[str, int],
    *,
    baudrate: int = DEFAULT_BAUDRATE,
    controller: Optional["ServoControler"] = None,
) -> Dict[str, object]:
    missing = [joint for joint in ARM_JOINT_ORDER if joint not in joint_to_current_id]
    if missing:
        raise ValueError(f"Missing assignments for: {', '.join(missing)}")

    normalized_mapping = {
        joint: int(joint_to_current_id[joint])
        for joint in ARM_JOINT_ORDER
    }
    current_ids = list(normalized_mapping.values())
    if len(set(current_ids)) != len(current_ids):
        raise ValueError("Each joint must map to a unique motor ID")

    target_ids = {joint: ARM_SERVO_MAP[joint] for joint in ARM_JOINT_ORDER}
    all_reserved_ids = set(current_ids) | set(target_ids.values()) | set(WHEEL_SERVO_IDS)
    free_temp_ids = [motor_id for motor_id in range(250, 19, -1) if motor_id not in all_reserved_ids]
    if len(free_temp_ids) < len(ARM_JOINT_ORDER):
        raise RuntimeError("Not enough temporary IDs available to remap motors safely")

    rename_plan = []
    for index, joint in enumerate(ARM_JOINT_ORDER):
        current_id = normalized_mapping[joint]
        target_id = target_ids[joint]
        temp_id = free_temp_ids[index]
        rename_plan.append((joint, current_id, temp_id, target_id))

    with _calibration_bus(
        port,
        list(current_ids) + [temp_id for _, _, temp_id, _ in rename_plan],
        baudrate=baudrate,
        controller=controller,
    ) as bus:
        for _, current_id, temp_id, _ in rename_plan:
            if current_id == temp_id:
                continue
            bus.disable_torque(current_id)
            bus.write("ID", current_id, temp_id)
            bus.motors.pop(current_id, None)
            bus.motors[temp_id] = Motor(temp_id, SERVO_MODEL, MotorNormMode.RANGE_M100_100)

        for _, _, temp_id, target_id in rename_plan:
            bus.disable_torque(temp_id)
            bus.write("ID", temp_id, target_id)
            bus.motors.pop(temp_id, None)
            bus.motors[target_id] = Motor(target_id, SERVO_MODEL, MotorNormMode.RANGE_M100_100)

    return {
        "assigned_ids": target_ids,
        "source_mapping": normalized_mapping,
    }




class ServoControler:
    """Controller for wheels (7-9) and arm (1-6)."""

    # Default lerobot calibration directory
    LEROBOT_CALIBRATION_DIR = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "robots"

    def __init__(
        self,
        right_arm_wheel_usb: str = None,
        *,
        speed: int = DEFAULT_SPEED,
        action_map: Optional[Mapping[str, Mapping[int, int]]] = None,
        enable_arm: bool = False,
        arm_calibration_id: str = "xlerobot_arm",
    ) -> None:
        self.right_arm_wheel_usb = right_arm_wheel_usb
        self.speed = speed
        self.action_map = ACTION_MAP if action_map is None else action_map
        self._wheel_ids = tuple(sorted(next(iter(self.action_map.values())).keys()))
        self._arm_ids = tuple(sorted(ARM_SERVO_MAP.values()))
        
        self._arm_positions = {}
        self._arm_enabled = False
        self.wheel_bus = None
        self._bus_lock = threading.Lock()  # Prevent concurrent bus access

        if right_arm_wheel_usb:
            motors = {
                7: Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                8: Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                9: Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
                10: Motor(10, "sts3215", MotorNormMode.RANGE_M100_100),
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
                        10: Motor(10, "sts3215", MotorNormMode.RANGE_M100_100),
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

    def get_wheel_positions(self) -> Dict[int, int]:
        """Read the current position (steps) from wheel motors."""
        if not self.wheel_bus:
            return {}
        try:
            return self.wheel_bus.sync_read("Present_Position", list(self._wheel_ids))
        except Exception:
            return {}



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
        if not self._bus_lock.acquire(blocking=False):
            return {}  # Skip if bus is busy
        try:
            raw = self.wheel_bus.sync_read("Present_Position", list(self._arm_ids))
        except Exception:
            return {}
        finally:
            self._bus_lock.release()
            
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
            with self._bus_lock:
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
        angle = -66.0 if closed else 80.0
        return self.set_arm_joint("gripper", angle)

    # Stall Detection

    def get_arm_loads(self) -> Dict[int, int]:
        """Read the current load (0-1000) from arm motors."""
        if not self._arm_enabled:
            return {}
        if not self._bus_lock.acquire(blocking=False):
            return {}  # Skip if bus is busy
        try:
            return self.wheel_bus.sync_read("Present_Load", list(self._arm_ids))
        except Exception:
            return {}
        finally:
            self._bus_lock.release()

    def check_stall(self, threshold: int = 600) -> Optional[str]:
        """
        Check for stalled motors (load > threshold).
        If stalled, disabling torque for safety.
        Returns description of stall or None.
        """
        warnings = []
        
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

    def __del__(self) -> None:
        if hasattr(self, "wheel_bus") and self.wheel_bus and self.wheel_bus.is_connected:
            self.disconnect()
