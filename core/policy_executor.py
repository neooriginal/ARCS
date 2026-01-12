import logging
import threading
import time
import torch
import numpy as np
from pathlib import Path
from typing import Optional

from lerobot.policies.act.modeling_act import ACTPolicy
from state import state

logger = logging.getLogger(__name__)

POLICY_ROOT = Path("logs/policies")

class PolicyExecutor:
    def __init__(self):
        self.policy: Optional[ACTPolicy] = None
        self.device = "cpu"
        self.is_running = False
        self.thread = None
        self.current_policy_name = None
        self._lock = threading.Lock()

    def load_policy(self, policy_name: str, device: str = "auto"):
        policy_path = POLICY_ROOT / policy_name / "checkpoints" / "last" / "pretrained_model"
        
        # Auto-detect device
        if device == "auto" or not device:
            if torch.cuda.is_available():
                device = "cuda"
            elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
                device = "mps"
            else:
                device = "cpu"
        
        # Check if local path exists
        if policy_path.exists():
            load_source = str(policy_path.absolute())
            logger.info(f"Loading local policy from {load_source} on {device}")
        else:
            # Assume it's a HuggingFace Hub ID
            load_source = policy_name
            logger.info(f"Local path not found. Attempting to load '{policy_name}' from HuggingFace Hub on {device}")

        try:
            self.policy = ACTPolicy.from_pretrained(load_source)
            self.policy.to(device)
            self.policy.eval()
            self.current_policy_name = policy_name
            self.device = device
            logger.info("Policy loaded successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to load policy: {e}")
            self.policy = None
            return False

    def start_execution(self):
        with self._lock:
            if not self.policy:
                logger.error("No policy loaded")
                return False
            if self.is_running:
                return False
            
            self.is_running = True
            state.ai_enabled = True
            state.ai_status = f"Running Policy: {self.current_policy_name}"
            
            # Move head to homing position for consistent inference
            from core.config_manager import get_config
            if state.controller:
                homing_yaw = get_config("HEAD_HOMING_YAW", 0.0)
                homing_pitch = get_config("HEAD_HOMING_PITCH", 45.0)
                state.controller.turn_head_yaw(homing_yaw)
                state.controller.turn_head_pitch(homing_pitch)
                time.sleep(0.9)
            
            self.thread = threading.Thread(target=self._inference_loop, daemon=True)
            self.thread.start()
            return True

    def stop_execution(self):
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        with self._lock:
            state.ai_enabled = False
            state.stop_all_movement()
        return True

    def _inference_loop(self):
        dt = 0.05 # 20Hz
        try:
            self.policy.reset() 
        except:
            pass # Some policies don't need reset
        
        while self.is_running:
            loop_start = time.time()
            
            try:
                if not state.robot_system:
                    time.sleep(0.1)
                    continue

                frame_main = state.robot_system.get_frame()
                if frame_main is None:
                    continue
                    
                img_tensor = torch.from_numpy(frame_main).permute(2, 0, 1).float() / 255.0
                img_tensor = img_tensor.unsqueeze(0).to(self.device)
                
                # Check for right camera
                frame_right = state.robot_system.get_right_frame()
                img_right_tensor = None
                if frame_right is not None:
                    img_right_tensor = torch.from_numpy(frame_right).permute(2, 0, 1).float() / 255.0
                    img_right_tensor = img_right_tensor.unsqueeze(0).to(self.device)
                
                if not state.controller:
                     continue
                     
                arm_pos = state.controller.get_arm_position()
                current_joints = [
                    arm_pos.get('shoulder_pan', 0),
                    arm_pos.get('shoulder_lift', 0),
                    arm_pos.get('elbow_flex', 0),
                    arm_pos.get('wrist_flex', 0),
                    arm_pos.get('wrist_roll', 0),
                    arm_pos.get('gripper', 0)
                ]
                state_tensor = torch.tensor(current_joints, dtype=torch.float32).unsqueeze(0).to(self.device)

                batch = {
                    "observation.images.main": img_tensor,
                    "observation.state": state_tensor
                }
                
                if img_right_tensor is not None:
                    batch["observation.images.right"] = img_right_tensor
                
                with torch.inference_mode():
                    # select_action handles temporal ensembling internally
                    action_chunk = self.policy.select_action(batch) 
                
                action = action_chunk.squeeze(0).cpu().numpy()
                
                target_pos = {
                    'shoulder_pan': action[0],
                    'shoulder_lift': action[1],
                    'elbow_flex': action[2],
                    'wrist_flex': action[3],
                    'wrist_roll': action[4],
                    'gripper': action[5]
                }
                
                if state.controller:
                    state.controller.set_arm_position(target_pos)
            
            except Exception as e:
                logger.error(f"Inference Loop Error: {e}")
                time.sleep(1.0) # Prevent tight loop error spam

            elapsed = time.time() - loop_start
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)

# Singleton
policy_executor = PolicyExecutor()
