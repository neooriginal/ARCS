import logging
import threading
import subprocess
import queue
import time
import shutil
import torch
from pathlib import Path
from typing import Dict, Optional, List
from huggingface_hub import HfApi


DATASET_ROOT = Path("logs/datasets")
POLICY_ROOT = Path("logs/policies")

logger = logging.getLogger(__name__)

def get_hf_username() -> Optional[str]:
    """Get the current HuggingFace username from the logged-in account."""
    try:
        api = HfApi()
        user_info = api.whoami()
        return user_info.get("name")
    except Exception as e:
        logger.warning(f"Could not get HuggingFace username: {e}")
        return None

class TrainingManager:
    def __init__(self):
        self.process = None
        self.logs_queue = queue.Queue()
        self.is_training = False
        self.current_job = None
        self.job_history = []
        self._monitor_thread = None

    def list_datasets(self) -> List[str]:
        if not DATASET_ROOT.exists():
            return []
        return [d.name for d in DATASET_ROOT.iterdir() if d.is_dir()]

    def list_policies(self) -> List[str]:
        if not POLICY_ROOT.exists():
            return []
        return [d.name for d in POLICY_ROOT.iterdir() if d.is_dir()]

    def push_dataset_to_hub(self, dataset_name: str) -> tuple:
        """Attempt to upload local dataset to Hub."""
        try:
             # Reuse DatasetRecorder logic or call command
             # Easier to invoke command
             dataset_path = DATASET_ROOT / dataset_name
             hf_user = get_hf_username()
             if not hf_user: return False, "No HF User"
             repo_id = f"{hf_user}/{dataset_name}"
             
             logger.info(f"Uploading {dataset_path} to {repo_id}...")
             cmd = ["huggingface-cli", "upload", repo_id, str(dataset_path), "--repo-type", "dataset", "--private"]
             res = subprocess.run(cmd, capture_output=True, text=True)
             if res.returncode == 0:
                 return True, "Uploaded"
             else:
                 return False, res.stderr
        except Exception as e:
            return False, str(e)

    def verify_dataset_on_hub(self, repo_id: str) -> bool:
        """Check if dataset exists on HuggingFace Hub private or public."""
        try:
            api = HfApi()
            # This raises error if not found/no access
            api.dataset_info(repo_id)
            return True
        except Exception:
            return False

    def delete_dataset(self, dataset_name: str) -> tuple:
        """Delete dataset locally and from HuggingFace Hub."""
        hf_username = get_hf_username()
        success_local = False
        success_hub = False
        msgs = []

        # 1. Delete Local
        dataset_path = DATASET_ROOT / dataset_name
        if dataset_path.exists():
            try:
                shutil.rmtree(dataset_path)
                msgs.append(f"Deleted local files for {dataset_name}.")
                success_local = True
            except Exception as e:
                msgs.append(f"Failed to delete local files: {e}")
        else:
            msgs.append("Local files not found (already deleted?).")
            success_local = True

        # 2. Delete from Hub
        if hf_username:
            repo_id = f"{hf_username}/{dataset_name}"
            try:
                api = HfApi()
                api.delete_repo(repo_id=repo_id, repo_type="dataset")
                msgs.append(f"Deleted remote repo {repo_id}.")
                success_hub = True
            except Exception as e:
                msgs.append(f"Failed to delete remote repo {repo_id} (might not exist): {e}")
        else:
            msgs.append("Could not delete from Hub (not logged in).")

        return True, " | ".join(msgs)

    def rename_dataset(self, old_name: str, new_name: str) -> tuple:
        """Rename dataset locally and on HuggingFace Hub."""
        hf_username = get_hf_username()
        msgs = []
        success = False

        # 1. Rename Local
        old_path = DATASET_ROOT / old_name
        new_path = DATASET_ROOT / new_name
        
        if not old_path.exists():
            return False, f"Dataset '{old_name}' not found locally."
        
        if new_path.exists():
            return False, f"Destination '{new_name}' already exists."

        try:
            old_path.rename(new_path)
            msgs.append(f"Renamed local folder to {new_name}.")
            success = True
        except Exception as e:
            return False, f"Failed to rename local folder: {e}"

        # 2. Rename on Hub
        if hf_username:
            old_repo_id = f"{hf_username}/{old_name}"
            new_repo_id = f"{hf_username}/{new_name}"
            
            try:
                api = HfApi()
                # Check if old repo exists
                try:
                    api.dataset_info(old_repo_id)
                    # Attempt move
                    api.move_repo(from_id=old_repo_id, to_id=new_repo_id, repo_type="dataset")
                    msgs.append(f"Renamed remote repo to {new_repo_id}.")
                except Exception as e:
                    # If it doesn't exist, just ignore
                    if "404" in str(e):
                        msgs.append("Remote repo not found (skipped).")
                    else:
                        msgs.append(f"Failed to rename remote repo: {e}")
            except Exception as e:
                msgs.append(f"Hub error: {e}")
        else:
            msgs.append("Hub rename skipped (not logged in).")

        return True, " | ".join(msgs)

    def start_training(self, dataset_name: str, job_name: str, device: str = "auto"):
        if self.is_training:
            return False, "Training already in progress"
        
        # Auto-detect device
        if device == "auto" or not device:
            if torch.cuda.is_available():
                device = "cuda"
            elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
                device = "mps"
            else:
                device = "cpu"

        # Get HuggingFace username to form proper repo_id
        hf_username = get_hf_username()
        if not hf_username:
            return False, "Not logged into HuggingFace. Run 'huggingface-cli login' first."
        
        repo_id = f"{hf_username}/{dataset_name}"
        
        # Verify dataset exists on Hub
        if not self.verify_dataset_on_hub(repo_id):
            # Try to upload it first
            logger.info(f"Dataset {repo_id} not found on Hub, attempting to upload...")
            success, msg = self.push_dataset_to_hub(dataset_name)
            if not success:
                return False, f"Dataset not on Hub and upload failed: {msg}. Please upload manually."
        
        # Create unique job name if not provided (though frontend force-provides it now)
        if not job_name:
            job_name = f"act_{dataset_name}_{int(time.time())}"
        
        output_dir = POLICY_ROOT / job_name
        
        import sys
        import os
        
        bin_dir = os.path.dirname(sys.executable)
        lerobot_train_bin = os.path.join(bin_dir, "lerobot-train")
        
        cmd = [
            lerobot_train_bin,
            f"--dataset.repo_id={repo_id}",
            "--policy.type=act",
            f"--output_dir={str(output_dir.absolute())}",
            f"--job_name={job_name}",
            f"--policy.device={device}",
            "--wandb.enable=false",
            "--policy.push_to_hub=false"
        ]
        
        logger.info(f"Starting training command: {' '.join(cmd)}")
        
        self.current_job = {
            "name": job_name,
            "dataset": dataset_name,
            "status": "starting",
            "start_time": time.time(),
            "cmd": " ".join(cmd)
        }
        self.is_training = True
        
        # Start subprocess
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            # Start monitoring thread
            self._monitor_thread = threading.Thread(target=self._monitor_training, daemon=True)
            self._monitor_thread.start()
            
            return True, job_name
            
        except Exception as e:
            self.is_training = False
            self.current_job = None
            logger.error(f"Failed to start training: {e}")
            return False, str(e)

    def stop_training(self):
        if self.process and self.is_training:
            self.process.terminate()
            # Wait a bit then kill if needed
            self.current_job['status'] = "cancelled"
            self.is_training = False
            return True
        return False

    def _monitor_training(self):
        if not self.process:
            return
            
        self.current_job['status'] = "running"
        logger.info(f"Training job {self.current_job['name']} started monitoring.")
        
        try:
            # Read lines
            for line in iter(self.process.stdout.readline, ''):
                if line:
                    stripped = line.strip()
                    self.logs_queue.put(stripped)
                    logger.info(f"[TRAIN] {stripped}")
            
            self.process.stdout.close()
            return_code = self.process.wait()
            
            self.is_training = False
            if return_code == 0:
                self.current_job['status'] = "completed"
                logger.info("Training finished successfully")
            else:
                self.current_job['status'] = "failed"
                logger.error(f"Training failed with exit code {return_code}")
 
        except Exception as e:
            logger.error(f"Error in training monitor: {e}")
            self.current_job['status'] = "failed"
            self.is_training = False
            
        # Move to history
        self.job_history.append(self.current_job)

    def get_status(self):
         return {
             "is_training": self.is_training,
             "current_job": self.current_job,
             "history": self.job_history[-5:] # Last 5
         }


    def get_logs(self):
        # Flush queue
        logs = []
        try:
            while True:
                line = self.logs_queue.get_nowait()
                logs.append(line)
        except queue.Empty:
            pass
        return logs
    
    def hf_login(self, token: str) -> tuple:
        """Login to HuggingFace cli."""
        try:
            cmd = ["huggingface-cli", "login", "--token", token, "--add-to-git-credential"]
            # Check OS, windows might need different credential logic, but standard cli works usually.
            res = subprocess.run(cmd, capture_output=True, text=True)
            if res.returncode == 0:
                user = get_hf_username()
                if user:
                    return True, f"Logged in as {user}"
                else:
                    return True, "Login successful (Username check failed)"
            else:
                return False, res.stderr
        except Exception as e:
            return False, str(e)

    def hf_logout(self) -> tuple:
        """Logout from HuggingFace."""
        try:
            cmd = ["huggingface-cli", "logout"]
            res = subprocess.run(cmd, capture_output=True, text=True)
            if res.returncode == 0:
                return True, "Logged out"
            else:
                return False, res.stderr
        except Exception as e:
            return False, str(e)
            
    def get_hf_user(self) -> Optional[str]:
        return get_hf_username()

# Singleton
training_manager = TrainingManager()
