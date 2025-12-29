"""VR Kinematics - PyBullet IK/FK for arm teleoperation."""

import numpy as np
import pybullet as p
from typing import Tuple
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
NUM_JOINTS = 6
NUM_IK_JOINTS = 3
WRIST_FLEX_INDEX = 3
WRIST_ROLL_INDEX = 4
GRIPPER_INDEX = 5
END_EFFECTOR_LINK_NAME = "Fixed_Jaw_tip"


def get_urdf_path() -> str:
    return str(Path(__file__).parent.parent / "robots" / "urdf" / "SO100" / "so100.urdf")


class ForwardKinematics:
    def __init__(self, client, robot_id: int, joint_indices: list, ee_link: int):
        self.client = client
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.ee_link = ee_link
    
    def compute(self, angles_deg: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.client is None:
            return np.array([0.2, 0.0, 0.15]), np.array([0, 0, 0, 1])
        
        angles = angles_deg.copy()
        angles[5] = 0.0
        angles_rad = np.deg2rad(angles)
        
        for i in range(NUM_JOINTS):
            if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                p.resetJointState(self.robot_id, self.joint_indices[i], angles_rad[i])
        
        state = p.getLinkState(self.robot_id, self.ee_link)
        return np.array(state[0]), np.array(state[1])


class IKSolver:
    def __init__(self, client, robot_id: int, joint_indices: list, ee_link: int,
                 limits_min: np.ndarray, limits_max: np.ndarray):
        self.client = client
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.ee_link = ee_link
        self.limits_min = limits_min
        self.limits_max = limits_max
        
        self.ik_lower = np.deg2rad(limits_min[:NUM_IK_JOINTS])
        self.ik_upper = np.deg2rad(limits_max[:NUM_IK_JOINTS])
        self.ik_ranges = self.ik_upper - self.ik_lower
    
    def solve(self, target: np.ndarray, current_deg: np.ndarray) -> np.ndarray:
        if self.client is None:
            return current_deg[:NUM_IK_JOINTS]
        
        angles = current_deg.copy()
        angles[5] = 0.0
        angles_rad = np.deg2rad(angles)
        
        for i in range(NUM_JOINTS):
            if i < len(self.joint_indices) and self.joint_indices[i] is not None:
                p.resetJointState(self.robot_id, self.joint_indices[i], angles_rad[i])
        
        rest = np.deg2rad(current_deg[:NUM_IK_JOINTS])
        
        try:
            result = p.calculateInverseKinematics(
                self.robot_id, self.ee_link, target.tolist(),
                lowerLimits=self.ik_lower.tolist(),
                upperLimits=self.ik_upper.tolist(),
                jointRanges=self.ik_ranges.tolist(),
                restPoses=rest.tolist(),
                solver=0, maxNumIterations=100, residualThreshold=1e-4
            )
            solution = np.clip(
                np.rad2deg(result[:NUM_IK_JOINTS]),
                self.limits_min[:NUM_IK_JOINTS],
                self.limits_max[:NUM_IK_JOINTS]
            )
            return solution
        except Exception as e:
            logger.warning(f"IK failed: {e}")
            return current_deg[:NUM_IK_JOINTS]


class VRKinematics:
    def __init__(self):
        self.client = None
        self.robot_id = None
        self.joint_indices = []
        self.ee_link = None
        self.limits_min = np.full(NUM_JOINTS, -120.0)
        self.limits_max = np.full(NUM_JOINTS, 120.0)
        self.fk = None
        self.ik = None
        self.current_angles = np.zeros(NUM_JOINTS)
        self.is_initialized = False
    
    def initialize(self) -> bool:
        try:
            self.client = p.connect(p.DIRECT)
            urdf = get_urdf_path()
            if not Path(urdf).exists():
                logger.error(f"URDF not found: {urdf}")
                return False
            
            self.robot_id = p.loadURDF(urdf, useFixedBase=True)
            n = p.getNumJoints(self.robot_id)
            self.joint_indices = [None] * NUM_JOINTS
            
            for i in range(n):
                info = p.getJointInfo(self.robot_id, i)
                name = info[1].decode('utf-8')
                link = info[12].decode('utf-8')
                
                for idx, jname in enumerate(JOINT_NAMES):
                    if jname in name.lower() or str(idx + 1) == name:
                        self.joint_indices[idx] = i
                        if info[8] < info[9]:
                            self.limits_min[idx] = np.rad2deg(info[8])
                            self.limits_max[idx] = np.rad2deg(info[9])
                        break
                
                if END_EFFECTOR_LINK_NAME in link:
                    self.ee_link = i
            
            if self.ee_link is None:
                self.ee_link = n - 1
            
            self.fk = ForwardKinematics(self.client, self.robot_id, self.joint_indices, self.ee_link)
            self.ik = IKSolver(self.client, self.robot_id, self.joint_indices, self.ee_link,
                              self.limits_min, self.limits_max)
            
            self.is_initialized = True
            logger.info("VR Kinematics initialized")
            return True
        except Exception as e:
            logger.error(f"VR Kinematics init failed: {e}")
            return False
    
    def get_end_effector_position(self, angles: np.ndarray = None) -> np.ndarray:
        if not self.is_initialized:
            return np.array([0.2, 0.0, 0.15])
        if angles is None:
            angles = self.current_angles
        pos, _ = self.fk.compute(angles)
        return pos
    
    def solve_ik(self, target: np.ndarray, current: np.ndarray = None) -> np.ndarray:
        if not self.is_initialized:
            return np.zeros(NUM_IK_JOINTS)
        if current is None:
            current = self.current_angles
        return self.ik.solve(target, current)
    
    def update_current_angles(self, angles: np.ndarray):
        self.current_angles = angles.copy()
    
    def cleanup(self):
        if self.client is not None:
            try:
                p.disconnect(self.client)
            except Exception:
                pass
            self.client = None
            self.is_initialized = False


def vr_to_robot_coordinates(vr_pos: dict, scale: float = 1.0) -> np.ndarray:
    """
    Convert VR controller position to robot coordinate system.
    
    VR coordinate system: X=right, Y=up, Z=back (towards user)
    Robot coordinate system: X=forward, Y=left, Z=up
    """
    return np.array([
        -vr_pos['x'] * scale,   # VR +Z (back) -> Robot +X (forward)
        vr_pos['z'] * scale,    # VR +X (right) -> Robot -Y (right) 
        vr_pos['y'] * scale     # VR +Y (up) -> Robot +Z (up)
    ])


def compute_relative_position(current: dict, origin: dict, scale: float = 1.0) -> np.ndarray:
    """Compute relative position from VR origin to current position."""
    delta = {
        'x': current['x'] - origin['x'],
        'y': current['y'] - origin['y'], 
        'z': current['z'] - origin['z']
    }
    return vr_to_robot_coordinates(delta, scale)


vr_kinematics = VRKinematics()
