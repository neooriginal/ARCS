# 🛠️ Setup Guide

## Prerequisites

- **Python**: 3.10+
- **Hardware**:
    - **Robot**: So101 Follower Robot Arm (or compatible `xlerobot_arm`)
    - **Vision**: USB Camera or Raspberry Pi Camera
- **API Key**: [OpenAI](https://platform.openai.com) key for AI features.

## 📦 One-Line Installation

**Linux / macOS:**
```bash
curl -fsSL https://raw.githubusercontent.com/neooriginal/ARCS/main/install.sh | bash
```

**Windows (PowerShell):**
```powershell
irm https://raw.githubusercontent.com/neooriginal/ARCS/main/install.ps1 | iex
```

The installer will:
1. Clone the repository to `~/ARCS` (or `%USERPROFILE%\ARCS` on Windows)
2. Clone hardware drivers
3. Create a Python virtual environment
4. Install all dependencies

## ⚙️ Configuration

**OpenAI API Key**:
1. Start the server: `python main.py`
2. Go to **Settings** > **AI Configuration**.
3. Enter your key (`sk-...`) and click Save.

## ⚙️ Detailed Settings

1. Start the server: `python main.py`
2. Open [http://localhost:5000/settings](http://localhost:5000/settings)
3. Configure:
   - **Hardware Ports**: Camera, Wheel USB, Head USB, LIDAR Port
   - **TTS**: Enable/disable, audio device, accent
   - **Safety**: Stall thresholds, brightness limits, LIDAR thresholds
   - **Advanced**: Stream quality, VR settings

> [!TIP]
> The Settings page includes camera previews to help you select the correct device.

## 🔧 Calibration

**Critical Step**: You must calibrate the arm motors before first use.

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/robot_acm0 --robot.id=xlerobot_arm
```

> [!TIP]
> Use `ls /dev/tty*` to find your specific USB port if `/dev/robot_acm0` doesn't work.

### Faster ID Assignment on an Assembled Arm

If the arm servos do not already use IDs `1-6`, open **Settings** and use the **Arm ID Wizard**:

1. Click **Scan Bus** to find every servo on the shared wheel/arm bus.
2. Click **Identify** for a joint, then gently move only that joint for a few seconds.
3. Repeat for all six joints, then click **Assign IDs**.

The wizard temporarily releases torque, detects which servo moved the most, and then remaps all arm IDs in one safe pass using temporary IDs to avoid collisions. This avoids disconnecting each servo one-by-one just to assign IDs.

## 🚀 Running the System

```bash
cd ~/ARCS
source venv/bin/activate  # or .\venv\Scripts\Activate.ps1 on Windows
python main.py
```

- **Web Interface**: [http://localhost:5000](http://localhost:5000)
- **Robot HUD**: [http://localhost:5000/display](http://localhost:5000/display)
- **Settings**: [http://localhost:5000/settings](http://localhost:5000/settings)

## 🛠️ Manual Installation

If you prefer manual setup:

```bash
git clone https://github.com/neooriginal/ARCS.git
cd ARCS
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
# Edit .env to add your OPENAI_API_KEY
python main.py
```
