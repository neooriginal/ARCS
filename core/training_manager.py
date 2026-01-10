import json
import logging
import threading
import subprocess
import queue
import time
import shutil
import torch
from pathlib import Path
from typing import Any, Dict, Optional, List
from huggingface_hub import HfApi


DATASET_ROOT = Path("logs/datasets")
POLICY_ROOT = Path("logs/policies")
POLICY_METADATA_FILE = POLICY_ROOT / "policy_metadata.json"

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
        self.logs = [] # Persistent Log History
        self.is_training = False
        self.current_job = None
        self.job_history = []
        self._monitor_thread = None
        
        # Remote Worker State
        self.workers = {} 
        self.pending_jobs = queue.Queue()
        self.worker_logs = {}

    def register_worker_heartbeat(self, data: dict) -> dict:
        """Process heartbeat, return pending job if any."""
        wid = data['worker_id']
        now = time.time()
        
        if data.get('status') == 'offline':
             if wid in self.workers:
                 # If working, fail the job
                 if self.workers[wid].get("status") == "working":
                     job_name = self.workers[wid].get("job_name")
                     if job_name:
                         logger.warning(f"Worker {wid} disconnected while working on {job_name}")
                         self.remote_complete({"worker_id": wid, "job_name": job_name, "status": "failed", "error": "Worker disconnected"})
                 del self.workers[wid]
             return {"job_available": False}

        # Update Worker Info
        self.workers[wid] = {
            "last_seen": now,
            "gpu": data.get("gpu", "Unknown"),
            "platform": data.get("platform", "?"),
            "status": data.get("status", "idle"),
            "job_name": data.get("job_name")
        }
        
        # Assign job if idle
        if self.workers[wid]['status'] == 'idle' and not self.pending_jobs.empty():
            try:
                job = self.pending_jobs.get_nowait()
                self.current_job = job
                self.is_training = True
                self.workers[wid]['status'] = 'assigning'
                self.workers[wid]['job_name'] = job['name']
                return {"job_available": True, "job": job}
            except queue.Empty:
                pass
                
        return {"job_available": False}

    def cleanup_stale_workers(self, timeout: int = 300):
        """Remove workers/fail jobs if no heartbeat for timeout seconds."""
        now = time.time()
        to_remove = []
        for wid, info in self.workers.items():
            if now - info['last_seen'] > timeout:
                to_remove.append(wid)
                
        for wid in to_remove:
            info = self.workers[wid]
            if info.get('status') == 'working':
                job_name = info.get("job_name")
                if job_name:
                    logger.warning(f"Worker {wid} timed out while working on {job_name}")
                    self.remote_complete({"worker_id": wid, "job_name": job_name, "status": "failed", "error": "Worker timed out"})
            if wid in self.workers:
                del self.workers[wid]

    def get_worker_status(self):
        """Get list of active workers + trigger cleanup."""
        self.cleanup_stale_workers()
        params = []
        now = time.time()
        for wid, info in self.workers.items():
            params.append({
                "id": wid,
                "gpu": info['gpu'],
                "status": info['status'],
                "last_seen": int(now - info['last_seen'])
            })
        return params

    def remote_log(self, data: dict):
        job_name = data.get('job_name')
        wid = data.get('worker_id')
        line = data.get('log', '')
        
        # Refresh last seen for this worker
        if wid and wid in self.workers:
            self.workers[wid]['last_seen'] = time.time()
            self.workers[wid]['status'] = 'working'
            
        if self.current_job and self.current_job['name'] == job_name:
            # Check for cancellation signal
            if self.current_job.get('status') == 'cancelling':
                return {"abort": True}

            self.logs.append(line)
            # Update Device Info if not present
            if 'worker_info' not in self.current_job and wid in self.workers:
                 self.current_job['worker_info'] = self.workers[wid]['gpu']
            logger.info(f"[REMOTE] {line}")
            return {"abort": False}
        else:
             logger.debug(f"[REMOTE IGNORED] {job_name}: {line}")
             # If job mismatches (e.g. server restarted or job killed), tell worker to stop
             return {"abort": True}

    def remote_complete(self, data: dict):
        job_name = data.get('job_name')
        status = data.get('status')
        
        # Determine strict status
        if self.current_job and self.current_job.get('status') == 'cancelling':
             status = "cancelled"

        if self.current_job and self.current_job['name'] == job_name:
            self.is_training = False
            self.current_job['status'] = status
            logger.info(f"Remote job {job_name} finished: {status}")
            self.job_history.append(self.current_job)
            self.current_job = None

    def queue_remote_training(self, dataset_name: str, job_name: str, steps: int = 2000):
        """Queue a job for a remote worker."""
        if self.is_training:
            return False, "Training in progress"
            
        hf_user = get_hf_username()
        if not hf_user:
            return False, "Not logged in to HF"
        
        repo_id = f"{hf_user}/{dataset_name}"
        policy_repo = f"{hf_user}/{job_name}"
        
        # Using lerobot.scripts.lerobot_train based on pip show file list
        cmd = (
            f"python -m lerobot.scripts.lerobot_train "
            f"--dataset.repo_id={repo_id} "
            f"--policy.type=act "
            f"--policy.repo_id={policy_repo} "
            f"--job_name={job_name} "
            f"--policy.device=cuda "
            f"--steps={steps} "
            f"--save_freq=500 "
            f"--eval_freq=10000 "
            f"--log_freq=50 "
            f"--wandb.enable=false"
        )
        
        job = {
            "name": job_name,
            "dataset": dataset_name,
            "command": cmd,
            "status": "pending",
            "start_time": time.time(),
            "mode": "remote"
        }
        
        self.pending_jobs.put(job)
        # Set training state immediately so UI doesn't flicker
        self.is_training = True
        self.current_job = job
        self.logs = [] # Clear logs for new job
        return True, "Job Queued. Waiting for Worker..."

    def list_datasets(self) -> List[str]:
        if not DATASET_ROOT.exists():
            return []
        return [d.name for d in DATASET_ROOT.iterdir() if d.is_dir()]

    def _load_policy_metadata(self) -> Dict[str, Dict[str, Any]]:
        """Load policy metadata from JSON file."""
        if POLICY_METADATA_FILE.exists():
            try:
                with open(POLICY_METADATA_FILE, 'r') as f:
                    return json.load(f)
            except Exception as e:
                logger.warning(f"Failed to load policy metadata: {e}")
        return {}

    def _save_policy_metadata(self, metadata: Dict[str, Dict[str, Any]]) -> bool:
        """Save policy metadata to JSON file."""
        try:
            POLICY_ROOT.mkdir(parents=True, exist_ok=True)
            with open(POLICY_METADATA_FILE, 'w') as f:
                json.dump(metadata, f, indent=2)
            return True
        except Exception as e:
            logger.error(f"Failed to save policy metadata: {e}")
            return False

    def update_policy_metadata(self, policy_name: str, update_data: Dict[str, Any]) -> bool:
        """Update metadata (enabled, description) for a policy."""
        metadata = self._load_policy_metadata()
        if policy_name not in metadata:
            metadata[policy_name] = {'enabled': True, 'description': ''}
        metadata[policy_name].update(update_data)
        return self._save_policy_metadata(metadata)

    def list_policies(self) -> List[Dict[str, Any]]:
        """Return list of policy objects with name, enabled, and description."""
        policy_names = []
        
        # Local
        if POLICY_ROOT.exists():
            policy_names.extend([d.name for d in POLICY_ROOT.iterdir() if d.is_dir()])

        # Remote (HuggingFace)
        try:
            hf_user = get_hf_username()
            if hf_user:
                api = HfApi()
                models = api.list_models(author=hf_user, sort="lastModified", direction=-1, limit=10)
                for m in models:
                    policy_names.append(m.modelId)
        except Exception as e:
            logger.warning(f"Failed to list HF models: {e}")

        # Load metadata and build result
        metadata = self._load_policy_metadata()
        result = []
        for name in policy_names:
            meta = metadata.get(name, {})
            result.append({
                'name': name,
                'enabled': meta.get('enabled', True),
                'description': meta.get('description', '')
            })
        return result

    def get_policies_for_ai(self) -> List[Dict[str, str]]:
        """Return only enabled policies with name and description for AI tool generation."""
        all_policies = self.list_policies()
        return [
            {'name': p['name'], 'description': p['description']}
            for p in all_policies if p.get('enabled', True)
        ]

    def push_dataset_to_hub(self, dataset_name: str) -> tuple:
        """Attempt to upload local dataset to Hub."""
        try:
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

    def _delete_local_dataset(self, dataset_name: str) -> str:
        dataset_path = DATASET_ROOT / dataset_name
        if not dataset_path.exists():
            return "Local files not found (already deleted?)."
        try:
            shutil.rmtree(dataset_path)
            return f"Deleted local files for {dataset_name}."
        except Exception as e:
            return f"Failed to delete local files: {e}"

    def _delete_remote_dataset(self, repo_id: str) -> str:
        try:
            api = HfApi()
            api.delete_repo(repo_id=repo_id, repo_type="dataset")
            return f"Deleted remote repo {repo_id}."
        except Exception as e:
            return f"Failed to delete remote repo {repo_id} (might not exist): {e}"

    def delete_dataset(self, dataset_name: str) -> tuple[bool, str]:
        """Delete dataset locally and from HuggingFace Hub."""
        hf_username = get_hf_username()
        msgs = []

        msgs.append(self._delete_local_dataset(dataset_name))

        if hf_username:
            repo_id = f"{hf_username}/{dataset_name}"
            msgs.append(self._delete_remote_dataset(repo_id))
        else:
            msgs.append("Could not delete from Hub (not logged in).")

        return True, " | ".join(msgs)

    def _rename_local_dataset(self, old_name: str, new_name: str) -> tuple[bool, str]:
        old_path = DATASET_ROOT / old_name
        new_path = DATASET_ROOT / new_name
        
        if not old_path.exists():
            return False, f"Dataset '{old_name}' not found locally."
        if new_path.exists():
            return False, f"Destination '{new_name}' already exists."

        try:
            old_path.rename(new_path)
            return True, f"Renamed local folder to {new_name}."
        except Exception as e:
            return False, f"Failed to rename local folder: {e}"

    def _rename_remote_dataset(self, old_repo_id: str, new_repo_id: str) -> str:
        try:
            api = HfApi()
            try:
                api.dataset_info(old_repo_id)
                api.move_repo(from_id=old_repo_id, to_id=new_repo_id, repo_type="dataset")
                return f"Renamed remote repo to {new_repo_id}."
            except Exception as e:
                if "404" in str(e):
                    return "Remote repo not found (skipped)."
                return f"Failed to rename remote repo: {e}"
        except Exception as e:
             return f"Hub error: {e}"

    def rename_dataset(self, old_name: str, new_name: str) -> tuple[bool, str]:
        """Rename dataset locally and on HuggingFace Hub."""
        hf_username = get_hf_username()
        msgs = []

        success_local, msg_local = self._rename_local_dataset(old_name, new_name)
        msgs.append(msg_local)
        
        if not success_local:
             return False, msg_local

        if hf_username:
            old_repo_id = f"{hf_username}/{old_name}"
            new_repo_id = f"{hf_username}/{new_name}"
            msgs.append(self._rename_remote_dataset(old_repo_id, new_repo_id))
        else:
            msgs.append("Hub rename skipped (not logged in).")

        return True, " | ".join(msgs)

    def rename_policy(self, old_name: str, new_name: str) -> tuple[bool, str]:
        """Rename policy locally and on HuggingFace Hub."""
        hf_username = get_hf_username()
        msgs = []

        # Rename local folder if it exists
        old_path = POLICY_ROOT / old_name
        new_path = POLICY_ROOT / new_name
        if old_path.exists():
            if new_path.exists():
                return False, f"Destination '{new_name}' already exists."
            try:
                old_path.rename(new_path)
                msgs.append(f"Renamed local folder to {new_name}.")
            except Exception as e:
                return False, f"Failed to rename local folder: {e}"
        else:
            msgs.append("Local folder not found (might be remote only).")

        # Rename on HuggingFace Hub
        if hf_username:
            old_repo_id = f"{hf_username}/{old_name}"
            new_repo_id = f"{hf_username}/{new_name}"
            try:
                api = HfApi()
                api.model_info(old_repo_id)
                api.move_repo(from_id=old_repo_id, to_id=new_repo_id, repo_type="model")
                msgs.append(f"Renamed remote model to {new_repo_id}.")
            except Exception as e:
                if "404" in str(e):
                    msgs.append("Remote model not found (skipped).")
                else:
                    msgs.append(f"Failed to rename remote model: {e}")
        else:
            msgs.append("Hub rename skipped (not logged in).")

        # Update metadata file
        metadata = self._load_policy_metadata()
        if old_name in metadata:
            metadata[new_name] = metadata.pop(old_name)
            self._save_policy_metadata(metadata)
            msgs.append("Metadata updated.")

        return True, " | ".join(msgs)

    def start_training(self, dataset_name: str, job_name: str, device: str = "auto", steps: int = 2000):
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
            f"--steps={steps}",
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
        self.logs = [] # Clear logs
        
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
        # Handle Pending Remote Job
        if self.current_job and self.current_job.get('status') == 'pending':
            self.current_job['status'] = "cancelled"
            self.is_training = False
            # Clear pending queue
            with self.pending_jobs.mutex:
                self.pending_jobs.queue.clear()
            logger.info("Cancelled pending remote job.")
            return True

        # Handle Running Remote Job - Signal Cancellation
        if self.current_job and self.current_job.get('mode') == 'remote':
            self.current_job['status'] = "cancelling"
            logger.info("Signalling remote job cancellation...")
            return True

        # Handle Running Local Process
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
                    self.logs.append(stripped)
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

    def get_logs(self, since: int = 0):
        """Get logs starting from index `since`."""
        if since < 0: since = 0
        if since >= len(self.logs):
            return []
        return self.logs[since:]
    
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
