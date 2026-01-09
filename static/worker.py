"""ARCS Remote Worker - Connects to the Robot to run training jobs."""
import argparse
import requests
import time
import subprocess
import sys
import platform
import shlex
import os

VERSION = "1.1.1"

def get_worker_id():
    return f"worker_{platform.node()}_{os.getpid()}"

def get_gpu_info():
    try:
        if platform.system() == "Darwin":
            return "Apple Silicon (MPS)"
        res = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=5
        )
        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.strip()
    except Exception:
        pass
    return "CPU"

import atexit
import signal

SERVER_URL = None
WORKER_ID = None

def on_exit():
    if SERVER_URL and WORKER_ID:
        try:
            print("\n🔌 Disconnecting...")
            requests.post(f"{SERVER_URL}/api/worker/update", json={
                "worker_id": WORKER_ID, "status": "offline"
            }, timeout=2)
        except:
            pass

atexit.register(on_exit)
signal.signal(signal.SIGTERM, lambda n, f: sys.exit(0))
signal.signal(signal.SIGINT, lambda n, f: sys.exit(0))

def run_job(server_url, job, worker_id):
    job_name = job['name']
    
    requests.post(f"{server_url}/api/worker/update", json={
        "worker_id": worker_id, "status": "working", "job_name": job_name
    }, timeout=5)

    print("📦 Syncing Dataset from Hub...")
    
    # Robustly find lerobot training script
    try:
        import lerobot
        script_path = os.path.join(os.path.dirname(lerobot.__file__), 'scripts', 'lerobot_train.py')
        if not os.path.exists(script_path):
            raise FileNotFoundError(f"Script not found at {script_path}")
    except Exception as e:
        print(f"❌ Error finding lerobot: {e}")
        requests.post(f"{server_url}/api/worker/complete", json={
            "worker_id": worker_id, "job_name": job_name, "status": "failed", "error": f"LeRobot script not found: {e}"
        }, timeout=5)
        return

    # Construct the command using the safe script path
    # We parse the job['command'] to extract arguments
    # Expected original cmd: "python -m lerobot.scripts.lerobot_train --arg=val ..."
    original_cmd_str = job['command']
    
    # Quick parse to get flags
    if "--" in original_cmd_str:
        flags_str = original_cmd_str[original_cmd_str.index("--"):]
        flags = shlex.split(flags_str)
    else:
        flags = []

    # Final command: [python, script_path, *flags]
    cmd = [sys.executable, script_path] + flags
    
    print(f"🚀 Executing: {' '.join(cmd)}")

    # Force unbuffered output
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["HF_HUB_DISABLE_PROGRESS_BARS"] = "1" # Reduce noise

    try:
        process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, env=env
        )
        
        for line in iter(process.stdout.readline, ''):
            clean_line = line.strip()
            if clean_line:
                print(f"[train] {clean_line}")
                try:
                    requests.post(f"{server_url}/api/worker/log", json={
                        "worker_id": worker_id, "job_name": job_name, "log": clean_line
                    }, timeout=2)
                except Exception:
                    pass
                
        process.wait()
        final_status = "completed" if process.returncode == 0 else "failed"
        
        requests.post(f"{server_url}/api/worker/complete", json={
            "worker_id": worker_id, "job_name": job_name,
            "status": final_status, "return_code": process.returncode
        }, timeout=5)
        
    except Exception as e:
        print(f"❌ Job Failed: {e}")
        requests.post(f"{server_url}/api/worker/complete", json={
            "worker_id": worker_id, "job_name": job_name,
            "status": "failed", "error": str(e)
        }, timeout=5)

def main():
    print(f"🤖 ARCS Remote Worker v{VERSION}")
    
    global SERVER_URL, WORKER_ID
    worker_id = get_worker_id()
    WORKER_ID = worker_id
    gpu = get_gpu_info()
    device_name = f"{platform.system()} ({platform.machine()})"
    
    print(f"🆔 {worker_id}")
    print(f"🖥️  {device_name} | {gpu}")
    
    server_url = None
    if len(sys.argv) > 1:
        server_url = sys.argv[1]
    else:
        user_input = input("Robot URL (e.g. http://192.168.1.50:5000): ").strip()
        if user_input:
            server_url = user_input
    
    if not server_url:
        print("❌ No URL provided.")
        return
        
    if not server_url.startswith("http"):
        server_url = "http://" + server_url
    server_url = server_url.rstrip("/")
    SERVER_URL = server_url
    
    print(f"📡 {server_url}")
    print()

    while True:
        try:
            payload = {
                "worker_id": worker_id,
                "gpu": gpu,
                "platform": device_name,
                "status": "idle"
            }
            res = requests.post(f"{server_url}/api/worker/heartbeat", json=payload, timeout=5)
            
            if res.status_code == 200:
                print("\r✅ Ready. Waiting for jobs... ", end="", flush=True)
                data = res.json()
                
                if data.get("job_available"):
                    job = data['job']
                    print(f"\n🚀 JOB: {job['name']} (Dataset: {job['dataset']})")
                    run_job(server_url, job, worker_id)
                    print("\n🏁 Done. Resuming standby.")
            else:
                print(f"\r⚠️ Server {res.status_code}", end="", flush=True)
                
        except requests.exceptions.ConnectionError:
            print(f"\r🔌 Connecting to {server_url}...", end="", flush=True)
        except KeyboardInterrupt:
            # Trigger exit handlers
            sys.exit(0)
        except Exception as e:
            print(f"\n⚠️ {e}")
        
        time.sleep(3)

if __name__ == "__main__":
    main()
