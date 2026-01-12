import logging
import time
import threading
import subprocess
from pathlib import Path
from typing import Optional, Dict

import cv2
import numpy as np
import torch
from huggingface_hub import HfApi

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from state import state
from core.training_manager import get_hf_username

logger = logging.getLogger(__name__)

DATASET_ROOT = Path("logs/datasets")
FPS = 30

class DatasetRecorder:
    def __init__(self, main_camera, right_camera=None):
        self.main_camera = main_camera
        self.right_camera = right_camera
        self.is_recording = False
        self.dataset_name = ""
        self.episode_idx = 0
        self.frame_idx = 0
        self.dataset: Optional[LeRobotDataset] = None
        self.thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def start_recording(self, dataset_name: str) -> bool:
        with self._lock:
            if self.is_recording:
                logger.warning("Recording already in progress.")
                return False

            self.hf_username = get_hf_username()
            if not self.hf_username:
                logger.error("Not logged into HuggingFace. Run 'huggingface-cli login' first.")
                return False

            self.dataset_name = dataset_name
            self.dataset_dir = DATASET_ROOT / dataset_name
            self.repo_id = f"{self.hf_username}/{dataset_name}"

            try:
                if not self.dataset_dir.exists():
                    logger.info(f"Creating new dataset at {self.dataset_dir}")
                    self.dataset = LeRobotDataset.create(
                        repo_id=self.repo_id,
                        root=self.dataset_dir,
                        robot_type="so101_follower",
                        fps=FPS,
                        features={
                            "observation.images.main": {"dtype": "video", "shape": (3, 480, 640), "names": ["channel", "height", "width"]},
                            "observation.images.right": {"dtype": "video", "shape": (3, 480, 640), "names": ["channel", "height", "width"]},
                            "observation.state": {"dtype": "float32", "shape": (6,), "names": ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]},
                            "action": {"dtype": "float32", "shape": (6,), "names": ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]},
                        },
                        use_videos=True
                    )
                else:
                    logger.info(f"Loading existing dataset from {self.dataset_dir}")
                    self.dataset = LeRobotDataset(root=self.dataset_dir, repo_id=self.repo_id)
                    # Update episode index to next available
                    self.episode_idx = self.dataset.num_episodes

                if self.dataset is None:
                    logger.error("Dataset creation returned None")
                    return False

            except Exception as e:
                logger.error(f"Failed to create/load dataset: {e}")
                return False

            self.is_recording = True
            # Reset frame_idx for the *current* episode being recorded
            self.frame_idx = 0

            # Move head to homing position for consistent recording
            from core.config_manager import get_config
            if state.controller:
                homing_yaw = get_config("HEAD_HOMING_YAW", 0.0)
                homing_pitch = get_config("HEAD_HOMING_PITCH", 45.0)
                state.controller.turn_head_yaw(homing_yaw)
                state.controller.turn_head_pitch(homing_pitch)
                time.sleep(0.9)

            self.thread = threading.Thread(target=self._recording_loop, daemon=True)
            self.thread.start()
            logger.info(f"Started recording dataset: {self.repo_id} (Episode {self.episode_idx})")
            return True

    def stop_recording(self) -> bool:
        with self._lock:
            if not self.is_recording:
                return False

            self.is_recording = False
            if self.thread:
                self.thread.join(timeout=1.0)

            try:
                self._save_episode()
            except Exception as e:
                logger.error(f"Failed to save episode: {e}")

            if self.dataset is None:
                logger.error("Dataset object is None - cannot finalize or push")
                return False

            self._finalize_and_push()

            logger.info(f"Stopped recording. Dataset saved to {self.dataset_dir}")
            return True

    def _cli_upload_fallback(self) -> None:
        logger.info("Trying fallback upload with huggingface-cli...")
        try:
            result = subprocess.run(
                ["huggingface-cli", "upload", self.repo_id, str(self.dataset_dir),
                 "--repo-type", "dataset", "--private"],
                capture_output=True, text=True
            )
            if result.returncode == 0:
                logger.info(f"Fallback upload succeeded: https://huggingface.co/datasets/{self.repo_id}")
            else:
                logger.error(f"Fallback upload failed: {result.stderr}")
        except Exception as e:
            logger.error(f"Fallback upload error: {e}")

    def _finalize_and_push(self) -> None:
        try:
            logger.info("Finalizing dataset...")
            # Validate if finalize is needed for this library version; assuming yes based on usage
            self.dataset.finalize()
            logger.info("Dataset finalized successfully")
        except Exception as e:
            logger.error(f"Failed to finalize dataset: {e}")

        logger.info(f"Pushing dataset to HuggingFace Hub as {self.repo_id}...")
        try:
            self.dataset.push_to_hub(private=True)
            logger.info(f"Dataset pushed to Hub: https://huggingface.co/datasets/{self.repo_id}")
        except Exception as e:
            logger.error(f"push_to_hub failed: {e}")
            self._cli_upload_fallback()

    def _recording_loop(self) -> None:
        """High-frequency loop to capture data."""
        interval = 1.0 / FPS
        next_tick = time.time() + interval

        while self.is_recording:
            now = time.time()
            if now >= next_tick:
                self._capture_frame(now)
                next_tick += interval
            else:
                time.sleep(0.001)

    def _capture_frame(self, timestamp: float) -> None:
        controller = state.controller
        if not controller:
            if self.frame_idx == 0:
                logger.warning("Cannot capture: state.controller is None")
            return

        if not controller.arm_enabled:
            if self.frame_idx == 0:
                logger.warning("Cannot capture: arm not enabled on controller")
            return

        arm_pos = controller.get_arm_position()
        if not arm_pos:
            if self.frame_idx == 0:
                logger.warning("Cannot capture: get_arm_position() returned None/empty")
            return

        current_joints = [
            arm_pos.get('shoulder_pan', 0),
            arm_pos.get('shoulder_lift', 0),
            arm_pos.get('elbow_flex', 0),
            arm_pos.get('wrist_flex', 0),
            arm_pos.get('wrist_roll', 0),
            arm_pos.get('gripper', 0)
        ]

        action_joints = current_joints

        frame_main = None
        if state.latest_frame is not None:
            frame_resized = cv2.resize(state.latest_frame, (640, 480))
            frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            frame_main = torch.from_numpy(frame_rgb).permute(2, 0, 1)
        elif self.frame_idx == 0:
            logger.warning("Cannot capture: state.latest_frame is None (camera not running?)")
            return

        frame_right = None
        if hasattr(state, 'latest_frame_right') and state.latest_frame_right is not None:
            frame_right_resized = cv2.resize(state.latest_frame_right, (640, 480))
            frame_rgb_right = cv2.cvtColor(frame_right_resized, cv2.COLOR_BGR2RGB)
            frame_right = torch.from_numpy(frame_rgb_right).permute(2, 0, 1)

        frame_dict = {
            "observation.images.main": frame_main,
            "observation.state": torch.tensor(current_joints, dtype=torch.float32),
            "action": torch.tensor(action_joints, dtype=torch.float32),
            "task": "pick up object",
        }

        if frame_right is not None:
            frame_dict["observation.images.right"] = frame_right

        try:
            self.dataset.add_frame(frame_dict)
            self.frame_idx += 1
            if self.frame_idx % 100 == 0:
                logger.info(f"Captured {self.frame_idx} frames")
        except Exception as e:
            if self.frame_idx == 0:
                logger.error(f"add_frame failed: {e}")

    def on_episode_boundary(self) -> None:
        """Call this when VR user triggers a reset/new episode."""
        with self._lock:
            self._save_episode()
            self.episode_idx += 1

    def _save_episode(self) -> None:
        """Save the current episode using LeRobot v3.0 API."""
        if self.dataset is None:
            return

        if self.frame_idx == 0:
            logger.warning("No frames to save for this episode")
            return

        logger.info(f"Saving episode {self.episode_idx} with {self.frame_idx} frames")

        try:
            self.dataset.save_episode()
            logger.info(f"Episode {self.episode_idx} saved successfully")
        except Exception as e:
            logger.error(f"save_episode failed: {e}")
