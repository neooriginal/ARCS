# 🛠️ Setup Guide

## Prerequisites

- **Python**: 3.10+
- **Hardware**:
    - **Robot**: So101 Follower Robot Arm (or compatible `xlerobot_arm`)
    - **Vision**: USB Camera or Raspberry Pi Camera
- **API Key**: [OpenAI](https://platform.openai.com) key for GPT-5.1.

## 📦 Installation

1.  **Clone the Repository**
    ```bash
    git clone https://github.com/neooriginal/RoboCrew-ControlSystem.git
    cd RoboCrew-ControlSystem
    ```

2.  **Clone Robot Drivers**
    ```bash
    # Clone the custom branch required for hardware drivers
    git clone -b custom https://github.com/neooriginal/RoboCrew.git
    ```

3.  **Install Dependencies**
    ```bash
    pip install -r requirements.txt
    ```

## ⚙️ Configuration

1.  **Environment Setup**
    ```bash
    cp .env.example .env
    ```

2.  **Add API Key**
    Edit `.env` and paste your key:
    ```ini
    OPENAI_API_KEY=sk-your_actual_api_key_here
    ```

## 🔧 Calibration

**Critical Step**: You must calibrate the motors before first use.

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/robot_acm0 --robot.id=xlerobot_arm
```
> [!TIP]
> Use `ls /dev/tty*` to find your specific USB port if `/dev/robot_acm0` doesn't work.

## 🚀 Running the System

Start the control server:

```bash
python main.py
```

- **Web Interface**: [http://localhost:5000](http://localhost:5000)
- **Robot HUD**: [http://localhost:5000/display](http://localhost:5000/display)
