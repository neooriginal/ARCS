import base64
import cv2
from pathlib import Path
from langchain_core.tools import tool  # type: ignore[import]
from lerobot.async_inference.robot_client import RobotClient 
from lerobot.async_inference.configs import RobotClientConfig
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from robocrew.core.utils import capture_image
import time
import threading

from state import state as robot_state


def create_end_task():
    @tool
    def end_task(reason: str) -> str:
        """Call this when you have completed your assigned task or mission. Provide a reason explaining what was accomplished."""
        print(f"[TOOL] end_task - reason: {reason}")
        robot_state.ai_enabled = False
        robot_state.ai_status = f"Task completed: {reason}"
        robot_state.add_ai_log(f"TASK COMPLETED: {reason}")
        return f"Task ended. Reason: {reason}. AI has been paused."
    return end_task


def _interruptible_sleep(duration: float, check_interval: float = 0.1, check_safety: bool = False, movement_type: str = None):
    """
    Sleep that can be interrupted by emergency stop or SAFETY REFLEX.
    check_safety: If True, continuously checks ObstacleDetector.
    movement_type: 'FORWARD', 'BACKWARD', 'LEFT', 'RIGHT' for validation.
    """
    elapsed = 0
    # Safety Check Throttling (Don't check every 0.1s if not needed, but 10Hz is fine)
    
    while elapsed < duration:
        if not robot_state.ai_enabled:
            # Emergency stop - clear movement and exit
            robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
            return False
            
        # Update heartbeat to prevent watchdog from killing the movement
        robot_state.last_movement_activity = time.time()
        
        # --- CONTINUOUS SAFETY MONITORING ---
        if check_safety and movement_type == 'FORWARD' and robot_state.robot_system:
            try:
                # 1. Get Frame (Thread Safe)
                frame = robot_state.robot_system.get_frame()
                if frame is not None:
                    # 2. Check Detector (Shared History)
                    detector = robot_state.get_detector()
                    if detector:
                        safe_actions, _, _ = detector.process(frame)
                        if "FORWARD" not in safe_actions:
                            print("[SAFETY] EMERGENCY BRAKE: Obstacle appeared!")
                            robot_state.add_ai_log("SAFETY REFLEX: EMERGENCY STOP (Obstacle appeared)")
                            robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
                            return False
            except Exception as e:
                print(f"[SAFETY] Error during check: {e}")

        time.sleep(min(check_interval, duration - elapsed))
        elapsed += check_interval
    return True


def create_move_forward(servo_controller):
    @tool
    def move_forward(distance_meters: float) -> str:
        """Drives the robot forward for a specific distance."""
        distance = float(distance_meters)
        duration = abs(distance) / 0.15
        print(f"[TOOL] move_forward({distance}) for {duration:.1f}s")
        
        robot_state.movement = {'forward': True, 'backward': False, 'left': False, 'right': False}
        # Enable Continuous Safety Monitoring for forward movement
        completed = _interruptible_sleep(duration, check_safety=True, movement_type='FORWARD')
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
        
        if not completed:
            return "EMERGENCY STOP - Movement cancelled."
        return f"Moved forward {distance:.2f} meters."

    return move_forward

def create_move_backward(servo_controller):
    @tool
    def move_backward(distance_meters: float) -> str:
        """Drives the robot backward for a specific distance."""
        distance = float(distance_meters)
        duration = abs(distance) / 0.15
        print(f"[TOOL] move_backward({distance}) for {duration:.1f}s")
        
        robot_state.movement = {'forward': False, 'backward': True, 'left': False, 'right': False}
        completed = _interruptible_sleep(duration)
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
        
        if not completed:
            return "EMERGENCY STOP - Movement cancelled."
        return f"Moved backward {distance:.2f} meters."

    return move_backward


def create_turn_right(servo_controller):
    @tool
    def turn_right(angle_degrees: float) -> str:
        """Turns the robot right by angle in degrees."""
        angle = float(angle_degrees)
        duration = abs(angle) / 60
        print(f"[TOOL] turn_right({angle}) for {duration:.1f}s")
        
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': True}
        completed = _interruptible_sleep(duration)
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
        
        if not completed:
            return "EMERGENCY STOP - Movement cancelled."
        return f"Turned right by {angle} degrees."

    return turn_right


def create_turn_left(servo_controller):
    @tool
    def turn_left(angle_degrees: float) -> str:
        """Turns the robot left by angle in degrees."""
        angle = float(angle_degrees)
        duration = abs(angle) / 60
        print(f"[TOOL] turn_left({angle}) for {duration:.1f}s")
        
        robot_state.movement = {'forward': False, 'backward': False, 'left': True, 'right': False}
        completed = _interruptible_sleep(duration)
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
        
        if not completed:
            return "EMERGENCY STOP - Movement cancelled."
        return f"Turned left by {angle} degrees."

    return turn_left

def create_look_around(servo_controller, main_camera):
    @tool
    def look_around() -> list:
        """ONLY use this if you are completely stuck and need to find a new path. Looks left, center, right."""
        movement_delay = 0.8  # seconds
        print("Looking around...")
        servo_controller.turn_head_yaw(-30)  # Safe left
        time.sleep(movement_delay)
        image_left = capture_image(main_camera)
        image_left64 = base64.b64encode(image_left).decode('utf-8')
        servo_controller.turn_head_yaw(30)   # Safe right
        time.sleep(movement_delay)
        image_right = capture_image(main_camera)
        image_right64 = base64.b64encode(image_right).decode('utf-8')  
        servo_controller.turn_head_yaw(0)    # Center
        time.sleep(movement_delay)
        image_center = capture_image(main_camera)
        image_center64 = base64.b64encode(image_center).decode('utf-8')

        return [
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{image_left64}"}
                },
                {
                    "type": "image_url", 
                    "image_url": {"url": f"data:image/jpeg;base64,{image_center64}"}
                },
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{image_right64}"}
                }
            ]
        
    return look_around


def create_look_left(servo_controller):
    @tool
    def look_left() -> str:
        """Turn camera to look left (without moving the robot body)."""
        print("[TOOL] look_left - turning head left")
        servo_controller.turn_head_yaw(-60)
        time.sleep(0.5)
        return "Camera is now looking left. Use this view to check for openings or obstacles on your left side."
    return look_left


def create_look_right(servo_controller):
    @tool
    def look_right() -> str:
        """Turn camera to look right (without moving the robot body)."""
        print("[TOOL] look_right - turning head right")
        servo_controller.turn_head_yaw(60)
        time.sleep(0.5)
        return "Camera is now looking right. Use this view to check for openings or obstacles on your right side."
    return look_right


def create_look_center(servo_controller):
    @tool
    def look_center() -> str:
        """Reset camera to look straight ahead."""
        print("[TOOL] look_center - resetting head position")
        servo_controller.turn_head_yaw(0)
        servo_controller.turn_head_pitch(35)
        time.sleep(0.3)
        return "Camera is now looking straight ahead."
    return look_center


def create_look_down(servo_controller):
    @tool
    def look_down() -> str:
        """Tilt camera slightly down to see the ground closer to you."""
        print("[TOOL] look_down - tilting head down slightly")
        servo_controller.turn_head_pitch(50)
        time.sleep(0.3)
        return "Camera tilted down slightly. Check for obstacles on the ground ahead."
    return look_down


def create_look_up(servo_controller):
    @tool
    def look_up() -> str:
        """Tilt camera slightly up to see further ahead."""
        print("[TOOL] look_up - tilting head up slightly")
        servo_controller.turn_head_pitch(25)
        time.sleep(0.3)
        return "Camera tilted up slightly. You can now see further ahead."
    return look_up


def create_vla_single_arm_manipulation(
        tool_name: str,
        tool_description: str,
        task_prompt: str,
        server_address: str,
        policy_name: str, 
        policy_type: str, 
        arm_port: str,
        servo_controller, 
        camera_config: dict[str, dict], 
        main_camera_object,
        main_camera_usb_port: str,
        execution_time: int = 30,
        policy_device: str = "cuda"

    ):
    """Creates a tool that makes the robot pick up a cup using its arm.
    Args:
        server_address (str): The address of the server to connect to.
        policy_name (str): The name or path of the pretrained policy.
        policy_type (str): The type of policy to use.
        arm_port (str): The USB port of the robot's arm.
        camera_config (dict, optional): Lerobot-type camera configuration. (E.g., "{ main: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30}, left_arm: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}")
        policy_device (str, optional): The device to run the policy on. Defaults to "cuda".
    """
    configured_cameras = {}
    for cam_name, cam_settings in camera_config.items():
        # Unpack the dictionary settings directly into the Config class
        configured_cameras[cam_name] = OpenCVCameraConfig(
            index_or_path=cam_settings["index_or_path"],
            width=cam_settings.get("width", 640),
            height=cam_settings.get("height", 480),
            fps=cam_settings.get("fps", 30)
        )


    robot_config = SO101FollowerConfig(
        port=arm_port,
        cameras=configured_cameras,
        id="robot_arms",
        # TODO: Figure out calibration loading/saving issues
        # calibration_dir=Path("/home/pi/RoboCrew/calibrations")
    )

    cfg = RobotClientConfig(
        robot=robot_config,
        task=task_prompt,
        server_address=server_address,
        policy_type=policy_type,
        pretrained_name_or_path=policy_name,
        policy_device=policy_device,
        actions_per_chunk=50,
        chunk_size_threshold=0.5,
        fps=30
    )
    
    @tool
    def tool_name_to_override() -> str:
        """Tood description to override."""
        print("Manipulation tool activated")
        servo_controller.turn_head_pitch(45)
        servo_controller.turn_head_yaw(0)
        # release main camera from agent, so arm policy can use it
        main_camera_object.release()
        time.sleep(1)  # give some time to release camera

        try:
            client = RobotClient(cfg)
            if not client.start():
                return "Failed to connect to robot server."

            threading.Thread(target=client.receive_actions, daemon=True).start()
            threading.Timer(execution_time, client.stop).start()
            client.control_loop(task=task_prompt)
            
        
        finally:
            # Re-open main camera for agent use. 
            time.sleep(1)
            main_camera_object.open(main_camera_usb_port)
            main_camera_object.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            servo_controller.reset_head_position()
        
        return "Arm manipulation done"
    
    tool_name_to_override.name = tool_name
    tool_name_to_override.description = tool_description

    return tool_name_to_override


def create_check_alignment():
    @tool
    def check_gap_alignment() -> str:
        """
        Check for alignment with a potential gap or door in front.
        Use this tool ONLY when you intend to pass through a specific opening.
        Returns guidance (e.g., 'Turn LEFT slightly') or 'No specific gap detected'.
        """
        print("[TOOL] check_gap_alignment called")
        
        if robot_state.robot_system:
             try:
                frame = robot_state.robot_system.get_frame()
                if frame is not None:
                    detector = robot_state.get_detector()
                    if detector:
                        _, _, metrics = detector.process(frame)
                        guidance = metrics.get('guidance', '')
                        if guidance:
                            return f"ALIGNMENT SCAN RESULT: {guidance}"
                        else:
                            return "ALIGNMENT SCAN RESULT: No clear gap detected to align with. The path ahead might be open or fully blocked."
             except Exception as e:
                return f"Error checking alignment: {str(e)}"
        
        return "Error: Robot system not ready."
    return check_gap_alignment
