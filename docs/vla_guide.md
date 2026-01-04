# Vision-Language-Action (VLA) Guide

The RoboCrew system integrates **LeRobot** to enable Imitation Learning. This allows you to teach the robot tasks by demonstration (VR) and then train a policy to execute them autonomously.

## 1. Data Collection (VR)
**Files**: `core/dataset_recorder.py`, `templates/vr_control.html`

To train a policy, you first need a high-quality dataset of human demonstrations.

1.  **Enter VR**: Go to `/vr` on your Quest headset.
2.  **Set Name**: Enter a dataset name (e.g., `PourCoffee_v1`) in the UI overlay.
3.  **Record Episode**:
    *   Position the robot at the start state.
    *   Press **(A)** on the Right Controller (or "Record" on UI).
    *   **Red "REC" indicator** appears on the virtual screen.
    *   Perform the task smoothly.
    *   Press **(A)** again to stop.
4.  **Repeat**: Recording with the same name *appends* a new episode. Record 50-100 episodes for robust results.

> [!IMPORTANT]
> **Data Sync**: Each time you stop recording, the episode is saved locally (`logs/datasets/`) AND automatically pushed to your **HuggingFace Hub** account (if logged in).

## 2. Training (Imitate)
**Files**: `core/training_manager.py`, `templates/training.html`

Once you have data, train an **ACT (Action Chunking with Transformers)** policy.

1.  Navigate to **Training > Imitate**.
2.  **Select Dataset**: Find your dataset in the list.
3.  **Job Name**: Enter a unique name for this training run (e.g., `policy_pour_001`).
4.  **Hardware**: The system auto-detects your accelerator:
    *   **NVIDIA**: Uses `cuda` (Fastest)
    *   **Mac**: Uses `mps` (Fast)
    *   **Other**: Uses `cpu` (Slow)
5.  **Monitor**: View the live training logs in the dashboard.
6.  **Manage**: You can delete datasets (Local + Remote) using the 🗑️ icon.

## 3. Evaluation (Policy Execution)
**Files**: `core/policy_executor.py`

1.  In the **Trained Policies** list, click **Run**.
2.  The robot will take over control and attempt to replicate the task using visual feedback.

> [!NOTE]
> Ensure the environment matches the training conditions (lighting, object positions) for best results.
