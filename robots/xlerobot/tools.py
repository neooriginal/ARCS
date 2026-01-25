import base64
import cv2
from pathlib import Path
from typing import Optional
from langchain_core.tools import tool  # type: ignore[import]
from core.utils import capture_image
import time
import threading

from state import state as robot_state


def create_end_task():
    @tool
    def end_task(reason: str) -> str:
        """Call this when you have completed your assigned task or mission. Provide a reason explaining what was accomplished."""
        import tts
        
        print(f"[TOOL] end_task - reason: {reason}")
        robot_state.ai_enabled = False
        robot_state.precision_mode = False
        robot_state.ai_status = f"Task completed: {reason}"
        robot_state.add_ai_log(f"TASK COMPLETED: {reason}")
        
        # Ensure Approach Mode is disabled
        robot_state.approach_mode = False
        if robot_state.controller:
             robot_state.controller.set_speed(10000)
        
        # TTS Announcement
        tts.speak("Task complete")
             
        return f"Task ended. Reason: {reason}. AI has been paused."
    return end_task


def create_enable_precision_mode():
    @tool
    def enable_precision_mode() -> str:
        """Enable Precision Mode to see alignment targets for narrow gaps/doors."""
        robot_state.precision_mode = True
        return "Precision Mode ENABLED. You will now see target lines and alignment guidance on the video feed. Use this to align perfectly with the door."
    return enable_precision_mode

def create_disable_precision_mode():
    @tool
    def disable_precision_mode() -> str:
        """Disable Precision Mode to stop seeing alignment targets."""
        robot_state.precision_mode = False
        return "Precision Mode DISABLED. Alignment guidance hidden."
    return disable_precision_mode


def create_save_note():
    @tool
    def save_note(category: str, content: str) -> str:
        """Save a note to persistent memory about the environment. Use this to remember important layout details, landmarks, or observations. Categories: 'layout', 'landmark', 'obstacle', 'path', 'other'."""
        from core.memory_store import memory_store
        from state import state as robot_state
        
        note_id = memory_store.save_note(category, content, None)
        print(f"[TOOL] save_note({category}, {content[:50]}...) -> id={note_id}")
        return f"Note saved: [{category}] {content}"
    return save_note


def create_enable_approach_mode():
    @tool
    def enable_approach_mode() -> str:
        """Enable Approach Mode. Use this ONLY when you need to drive very close to a surface (counter, table) for manipulation. Disables standard safety stops."""
        import tts
        
        from state import state as robot_state
        robot_state.approach_mode = True
        robot_state.precision_mode = False
        
        # TTS Announcement
        tts.speak("Safety disabled")
             
        return "Approach Mode ENABLED. Safety thresholds relaxed. Speed limited to 10%."
    return enable_approach_mode

def create_disable_approach_mode():
    @tool
    def disable_approach_mode() -> str:
        """Disable Approach Mode. Re-enables standard safety stops."""
        from state import state as robot_state
        robot_state.approach_mode = False
        
        # Restore Speed (100%)
        if robot_state.controller:
             robot_state.controller.set_speed(10000)
             
        return "Approach Mode DISABLED. Safety systems active. Speed restored."
    return disable_approach_mode


def create_speak():
    @tool
    def speak(message: str) -> str:
        """Speak a message out loud via TTS. Use ONLY when necessary to communicate important information (task complete, warnings, errors). Do NOT use for routine status updates."""
        import tts
        print(f"[TOOL] speak: {message}")
        tts.speak(message)
        return f"Spoke: {message}"
    return speak


def _interruptible_sleep(duration: float, check_interval: float = 0.1, check_safety: bool = False, movement_type: str = None):
    """
    Sleep that can be interrupted by emergency stop or SAFETY REFLEX.
    check_safety: If True, continuously checks ObstacleDetector.
    movement_type: 'FORWARD', 'BACKWARD', 'LEFT', 'RIGHT' for validation.
    """
    elapsed = 0
    safety_check_interval = 0.2  # Check safety at 5Hz max
    last_safety_check = 0
    consecutive_blocks = 0  # Require multiple consecutive blocks for smoothing
    BLOCK_THRESHOLD = 3  # Require 3 consecutive blocked checks before stopping
    
    while elapsed < duration:
        if not robot_state.ai_enabled:
            # Emergency stop - clear movement and exit
            robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
            return False
            
        # Update heartbeat to prevent watchdog from killing the movement
        robot_state.last_movement_activity = time.time()
        
        # --- CONTINUOUS SAFETY MONITORING (throttled) ---
        current_time = time.time()
        if check_safety and movement_type == 'FORWARD' and robot_state.robot_system:
            if current_time - last_safety_check >= safety_check_interval:
                last_safety_check = current_time
                try:
                    frame = robot_state.robot_system.get_frame()
                    if frame is not None:
                        detector = robot_state.get_detector()
                        if detector:
                            safe_actions, _, _ = detector.process(frame)
                            if "FORWARD" not in safe_actions:
                                consecutive_blocks += 1
                                if consecutive_blocks >= BLOCK_THRESHOLD:
                                    print(f"[SAFETY] EMERGENCY BRAKE: Obstacle confirmed ({consecutive_blocks} checks)")
                                    robot_state.add_ai_log("SAFETY REFLEX: EMERGENCY STOP (Obstacle appeared)")
                                    robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
                                    return False
                            else:
                                consecutive_blocks = 0  # Reset on clear
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
        
        if robot_state.approach_mode:
            duration *= 10.0 # Slow speed compensation
            
        print(f"[TOOL] move_forward({distance}) for {duration:.1f}s (Approach={robot_state.approach_mode})")
        
        robot_state.movement = {'forward': True, 'backward': False, 'left': False, 'right': False}
        # Enable Continuous Safety Monitoring for forward movement
        completed = _interruptible_sleep(duration, check_safety=True, movement_type='FORWARD')
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
        
        if not completed:
            return "EMERGENCY STOP - Movement cancelled."
            
        # Check for Auto-Disable on Arrival (Approach Mode only)
        if robot_state.approach_mode and robot_state.robot_system:
             try:
                 frame = robot_state.robot_system.get_frame()
                 if frame is not None:
                     detector = robot_state.get_detector()
                     if detector:
                         _, _, metrics = detector.process(frame)
                         c_fwd = metrics.get('c_fwd', 0)
                         
                         # If extremely close (> 380), auto-disable
                         if c_fwd > 380:
                             robot_state.approach_mode = False
                             if robot_state.controller:
                                 robot_state.controller.set_speed(10000)
                             return f"Moved forward {distance:.2f} meters. ✓ TARGET REACHED (c_fwd={c_fwd}). Approach Mode Auto-Disabled. You are now touching/very close to the object."
                         
                         # If getting close (> 300), warn
                         elif c_fwd > 300:
                             return f"Moved forward {distance:.2f} meters. PROXIMITY WARNING: Very close (c_fwd={c_fwd}). One more small step should reach target."
             except Exception:
                 pass
                 
        return f"Moved forward {distance:.2f} meters."

    return move_forward

def create_move_backward(servo_controller):
    @tool
    def move_backward(distance_meters: float) -> str:
        """Drives the robot backward for a specific distance."""
        distance = float(distance_meters)
        duration = abs(distance) / 0.15
        
        if robot_state.approach_mode:
            duration *= 10.0 # Slow speed compensation
            
        print(f"[TOOL] move_backward({distance}) for {duration:.1f}s (Approach={robot_state.approach_mode})")
        
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
        # Minimum duration to overcome motor stiction
        MIN_DURATION = 0.15 
        calculated_duration = abs(angle) / 60
        duration = max(calculated_duration, MIN_DURATION)
        
        if robot_state.approach_mode:
            duration *= 10.0 # Slow speed compensation
        
        print(f"[TOOL] turn_right({angle}) -> dur={duration:.2f}s (Approach={robot_state.approach_mode})")
        
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
        # Minimum duration to overcome motor stiction
        MIN_DURATION = 0.15
        calculated_duration = abs(angle) / 60
        duration = max(calculated_duration, MIN_DURATION)
        
        if robot_state.approach_mode:
            duration *= 10.0 # Slow speed compensation
        
        print(f"[TOOL] turn_left({angle}) -> dur={duration:.2f}s (Approach={robot_state.approach_mode})")
        
        robot_state.movement = {'forward': False, 'backward': False, 'left': True, 'right': False}
        completed = _interruptible_sleep(duration)
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
        
        if not completed:
            return "EMERGENCY STOP - Movement cancelled."
        return f"Turned left by {angle} degrees."

    return turn_left
    
def create_slide_left(servo_controller):
    @tool
    def slide_left(distance_meters: float) -> str:
        """Slides the robot sideways to the left for a specific distance (Holonomic movement). Useful for aligning with objects without turning."""
        distance = float(distance_meters)
        duration = abs(distance) / 0.15
        
        if robot_state.approach_mode:
            duration *= 10.0 # Slow speed compensation
        
        print(f"[TOOL] slide_left({distance}) -> dur={duration:.2f}s (Approach={robot_state.approach_mode})")
        
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False, 'slide_left': True, 'slide_right': False}
        completed = _interruptible_sleep(duration)
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False, 'slide_left': False, 'slide_right': False}
        
        if not completed:
            return "EMERGENCY STOP - Movement cancelled."
        return f"Slid left by {distance:.2f} meters."

    return slide_left

def create_slide_right(servo_controller):
    @tool
    def slide_right(distance_meters: float) -> str:
        """Slides the robot sideways to the right for a specific distance (Holonomic movement). Useful for aligning with objects without turning."""
        distance = float(distance_meters)
        duration = abs(distance) / 0.15
        
        if robot_state.approach_mode:
            duration *= 10.0 # Slow speed compensation
        
        print(f"[TOOL] slide_right({distance}) -> dur={duration:.2f}s (Approach={robot_state.approach_mode})")
        
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False, 'slide_left': False, 'slide_right': True}
        completed = _interruptible_sleep(duration)
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False, 'slide_left': False, 'slide_right': False}
        
        if not completed:
            return "EMERGENCY STOP - Movement cancelled."
        return f"Slid right by {distance:.2f} meters."

    return slide_right





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
    from lerobot.async_inference.robot_client import RobotClient 
    from lerobot.async_inference.configs import RobotClientConfig
    from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from state import state as robot_state
    
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
        """Tool description to override."""
        controller = robot_state.controller
        if not controller:
            return "Error: Robot controller not ready."
            
        print("Manipulation tool activated")
        
        cam = robot_state.camera
        if cam:
             cam.release()
        time.sleep(1)

        try:
            client = RobotClient(cfg)
            if not client.start():
                return "Failed to connect to robot server."

            threading.Thread(target=client.receive_actions, daemon=True).start()
            threading.Timer(execution_time, client.stop).start()
            client.control_loop(task=task_prompt)
            
        
        finally:
            time.sleep(1)
            # Camera re-initialization would handle this on next usage attempt or requires manual reset
        
        return "Arm manipulation done"
    
    tool_name_to_override.name = tool_name
    tool_name_to_override.description = tool_description

    return tool_name_to_override


def create_run_robot_policy():
    from core.training_manager import training_manager

    # Build dynamic description with available policies
    policies = training_manager.get_policies_for_ai()
    policy_list = ""
    if policies:
        policy_list = "\n\nAvailable policies:\n" + "\n".join(
            f"  - {p['name']}: {p['description'] or 'No description'}"
            for p in policies
        )

    description = f"""Executes a trained robot policy to perform a physical task using the arms. 
Use this when the user asks to perform a specific learned skill (e.g. 'pickup_cup', 'wipe_table').
The policy will take control of the arms and base for the specified duration.{policy_list}"""

    @tool(description=description)
    def run_robot_policy(policy_name: str, duration_seconds: int = 45) -> str:
        from state import state
        from core.policy_executor import policy_executor
        from core.training_manager import training_manager
        import time

        print(f"[TOOL] run_robot_policy({policy_name}, duration={duration_seconds})")

        # Check if policy is enabled
        enabled_policies = training_manager.get_policies_for_ai()
        enabled_names = [p['name'] for p in enabled_policies]
        if policy_name not in enabled_names:
            return f"Error: Policy '{policy_name}' is disabled or does not exist. Available policies: {', '.join(enabled_names) or 'None'}"

        if not policy_executor.load_policy(policy_name):
            return f"Error: Failed to load policy '{policy_name}'. Check if it exists."

        if not policy_executor.start_execution():
            return "Error: Failed to start policy execution (maybe already running?)."

        step = 0.5
        elapsed = 0
        try:
            while elapsed < duration_seconds:
                if not policy_executor.is_running:
                    return "Policy execution stopped unexpectedly."
                if not state.ai_enabled:
                    policy_executor.stop_execution()
                    return "Policy execution interrupted by stop command."

                time.sleep(step)
                elapsed += step

        except Exception as e:
            policy_executor.stop_execution()
            return f"Error during policy execution: {e}"

        policy_executor.stop_execution()
        return f"Policy '{policy_name}' executed for {duration_seconds} seconds. Task should be complete."

    return run_robot_policy


def create_find_gap():
    @tool
    def find_gap() -> str:
        """
        Analyze 360° LIDAR data to find passable gaps in the forward direction (±90°).
        Use this in precision mode to find door openings without physically rotating.
        Returns gap location and alignment instructions.
        """
        from state import state as robot_state
        from core.lidar360 import get_lidar360
        
        lidar = get_lidar360()
        if not lidar or not lidar.connected:
            return "Error: 360° LIDAR not available. Is AI navigation enabled?"
        
        # Enable Precision Mode
        previous_mode = robot_state.precision_mode
        robot_state.precision_mode = True
        
        # Analyze forward arc (±90° from center)
        gap_info = lidar.find_gap_in_range(-90, 90)
        
        robot_state.last_scan_result = gap_info
        robot_state.precision_mode = previous_mode
        
        if not gap_info['found']:
            reason = gap_info.get('reason', 'unknown')
            return f"NO GAP FOUND in forward arc. Reason: {reason}. Try moving to a different position."
        
        center = gap_info['center_angle']
        width = gap_info['width_deg']
        
        # Determine alignment instruction
        if abs(center) < 5:
            alignment = "ALIGNED - Gap is directly ahead! Drive forward to pass through."
        elif center > 0:
            alignment = f"Turn LEFT {abs(center):.0f}° to align with gap center."
        else:
            alignment = f"Turn RIGHT {abs(center):.0f}° to align with gap center."
        
        return f"GAP FOUND: Center at {center:.1f}°, width {width:.1f}°. {alignment}"

    return find_gap


def create_align_to_gap():
    @tool
    def align_to_gap() -> str:
        """
        Scans for a gap using LIDAR and AUTOMATICALLY rotates the robot to face the center.
        Use this when close to a door/gap to quickly align before driving through.
        Returns the result of the alignment.
        """
        import time
        from state import state as robot_state
        from core.lidar360 import get_lidar360
        
        lidar = get_lidar360()
        if not lidar or not lidar.connected:
            return "Error: 360° LIDAR not available."
            
        # 1. Scan for gap
        previous_mode = robot_state.precision_mode
        robot_state.precision_mode = True
        
        # Analyze forward arc (±90°)
        gap_info = lidar.find_gap_in_range(-90, 90)
        
        robot_state.last_scan_result = gap_info
        
        if not gap_info['found']:
            robot_state.precision_mode = previous_mode
            reason = gap_info.get('reason', 'unknown')
            return f"NO GAP FOUND. Reason: {reason}. Try moving left/right manually."
            
        center_angle = gap_info['center_angle']
        width = gap_info['width_deg']
        
        # 2. Check alignment
        if abs(center_angle) < 5:
            robot_state.precision_mode = previous_mode
            return f"ALREADY ALIGNED (Error {center_angle:.1f}°). Gap is efficiently ahead. Drive forward."
            
        # 3. Auto-Rotate
        # Calculate duration based on ~60 deg/sec (same as turn_left tool)
        MIN_DURATION = 0.15
        duration = abs(center_angle) / 60.0
        duration = max(duration, MIN_DURATION)
        
        direction = "LEFT" if center_angle > 0 else "RIGHT"
        
        print(f"[TOOL] align_to_gap: Turning {direction} {abs(center_angle):.1f}° (Duration: {duration:.2f}s)")
        
        # Execute turn
        robot_state.movement = {
            'forward': False, 
            'backward': False, 
            'left': (direction == "LEFT"), 
            'right': (direction == "RIGHT")
        }
        
        completed = _interruptible_sleep(duration)
        
        # Stop
        robot_state.movement = {'forward': False, 'backward': False, 'left': False, 'right': False}
        robot_state.precision_mode = previous_mode
        
        if not completed:
            return "EMERGENCY STOP during auto-alignment."
            
        return f"SUCCESS: Found gap at {center_angle:.1f}° and auto-aligned {direction}. You should now be facing the gap (width {width:.1f}°). Drive forward."

    return align_to_gap
