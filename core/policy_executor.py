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
        
        try:
            logger.info(f"Loading policy from {policy_path} on {device}")
            self.policy = ACTPolicy.from_pretrained(str(policy_path.absolute()))
            self.policy.to(device)
            self.policy.eval()
            self.current_policy_name = policy_name
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
            
            self.thread = threading.Thread(target=self._inference_loop, daemon=True)
            self.thread.start()
            return True

    def stop_execution(self):
        with self._lock:
            self.is_running = False
            if self.thread:
                self.thread.join(timeout=1.0)
            state.ai_enabled = False
            state.stop_all_movement()
            return True

    def _inference_loop(self):
        dt = 0.05 # 20Hz
        self.policy.reset() 
        
        while self.is_running:
            loop_start = time.time()
            
            if not state.robot_system:
                time.sleep(0.1)
                continue

            frame_main = state.robot_system.get_frame()
            if frame_main is None:
                continue
                
            img_tensor = torch.from_numpy(frame_main).permute(2, 0, 1).float() / 255.0
            img_tensor = img_tensor.unsqueeze(0).to(self.policy.device)
            
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
            state_tensor = torch.tensor(current_joints, dtype=torch.float32).unsqueeze(0).to(self.policy.device)

            batch = {
                "observation.images.main": img_tensor,
                "observation.state": state_tensor
            }
            
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
            
            elapsed = time.time() - loop_start
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)

# Singleton
policy_executor = PolicyExecutor()
