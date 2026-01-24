"""
XLeRobot implementation of BaseRobot.

This module wraps the existing ServoControler to conform to the BaseRobot interface.
"""

from typing import Dict, Optional

from robots.base import BaseRobot
from robots.xlerobot.servo_controls import ServoControler


class XLeRobot(BaseRobot):
    """XLeRobot hardware driver implementing BaseRobot interface."""

    def __init__(
        self,
        wheel_usb: str,
        *,
        enable_arm: bool = False,
        arm_calibration_id: str = "xlerobot_arm",
    ) -> None:
        self._wheel_usb = wheel_usb
        self._enable_arm = enable_arm
        self._arm_calibration_id = arm_calibration_id
        self._controller: Optional[ServoControler] = None

    @property
    def name(self) -> str:
        return "XLeRobot"

    @property
    def has_wheels(self) -> bool:
        return True

    @property
    def has_arm(self) -> bool:
        return self._controller is not None and self._controller.arm_enabled

    def connect(self) -> None:
        self._controller = ServoControler(
            self._wheel_usb,
            enable_arm=self._enable_arm,
            arm_calibration_id=self._arm_calibration_id,
        )

    def disconnect(self) -> None:
        if self._controller:
            self._controller.disconnect()
            self._controller = None

    # --- Wheels ---

    def drive(self, forward: float, lateral: float = 0.0, rotation: float = 0.0) -> None:
        if self._controller:
            self._controller.set_velocity_vector(forward, lateral, rotation)

    def stop_wheels(self) -> None:
        if self._controller:
            self._controller._wheels_stop()

    def get_wheel_loads(self) -> Dict[int, int]:
        if self._controller:
            return self._controller.get_wheel_loads()
        return {}



    # --- Arm ---

    def set_arm_joints(self, positions: Dict[str, float]) -> Dict[str, float]:
        if self._controller:
            return self._controller.set_arm_position(positions)
        return {}

    def get_arm_joints(self) -> Dict[str, float]:
        if self._controller:
            return self._controller.get_arm_position()
        return {}

    def set_gripper(self, closed: bool) -> None:
        if self._controller:
            self._controller.set_gripper(closed)

    def get_arm_loads(self) -> Dict[int, int]:
        if self._controller:
            return self._controller.get_arm_loads()
        return {}

    # --- Safety ---

    def check_stall(self, threshold: int = 600) -> Optional[str]:
        if self._controller:
            return self._controller.check_stall(threshold)
        return None

    # --- Expose underlying controller for backwards compatibility ---

    @property
    def controller(self) -> Optional[ServoControler]:
        """Access underlying ServoControler for legacy code paths."""
        return self._controller
