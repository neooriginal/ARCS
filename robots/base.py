"""
Abstract base class for all robot hardware implementations.

This module defines the contract that any robot driver must implement.
"""

from abc import ABC, abstractmethod
from typing import Dict, Optional


class BaseRobot(ABC):
    """Base class for robot hardware drivers."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable name for this robot type."""
        pass

    @property
    def has_wheels(self) -> bool:
        """Whether this robot has wheeled movement."""
        return False

    @property
    def has_arm(self) -> bool:
        """Whether this robot has a manipulator arm."""
        return False

    @abstractmethod
    def connect(self) -> None:
        """Initialize hardware connection. Called once at startup."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Release hardware resources. Called at shutdown."""
        pass

    # --- Wheel/Movement ---

    def drive(self, forward: float, lateral: float = 0.0, rotation: float = 0.0) -> None:
        """
        Set wheel velocities.

        Args:
            forward: Forward/backward (-1.0 to 1.0)
            lateral: Side-to-side (-1.0 to 1.0, + is left)
            rotation: Turn in place (-1.0 to 1.0, + is left)
        """
        pass

    def stop_wheels(self) -> None:
        """Stop all wheel movement immediately."""
        pass

    def get_wheel_loads(self) -> Dict[int, int]:
        """Read current load from wheel motors."""
        return {}



    # --- Arm ---

    def set_arm_joints(self, positions: Dict[str, float]) -> Dict[str, float]:
        """
        Set arm joint positions.

        Args:
            positions: Dict of joint_name -> angle in degrees

        Returns:
            Updated joint positions
        """
        return {}

    def get_arm_joints(self) -> Dict[str, float]:
        """Get current arm joint positions."""
        return {}

    def set_gripper(self, closed: bool) -> None:
        """Open or close the gripper."""
        pass

    def get_arm_loads(self) -> Dict[int, int]:
        """Read current load from arm motors."""
        return {}

    # --- Safety ---

    def check_stall(self, threshold: int = 600) -> Optional[str]:
        """
        Check for stalled motors.

        Returns:
            Description of stall if detected, else None
        """
        return None
