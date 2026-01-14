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
                if frame is not None:
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

def create_look_around(servo_controller, main_camera):
    @tool
    def look_around() -> list:
        """ONLY use this if you are completely stuck and need to find a new path. Looks left, center, right."""
        # Use dynamic state
        controller = robot_state.controller
        camera = robot_state.camera
        
        if not controller or not camera:
             return "Error: Hardware (Head/Camera) not ready."
        
        movement_delay = 0.8  # seconds
        print("Looking around...")
        controller.turn_head_yaw(-30)  # Safe left
        time.sleep(movement_delay)
        image_left = capture_image(camera)
        image_left64 = base64.b64encode(image_left).decode('utf-8')
        controller.turn_head_yaw(30)   # Safe right
        time.sleep(movement_delay)
        image_right = capture_image(camera)
        image_right64 = base64.b64encode(image_right).decode('utf-8')  
        controller.turn_head_yaw(0)    # Center
        time.sleep(movement_delay)
        image_center = capture_image(camera)
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
        controller.turn_head_pitch(45)
        controller.turn_head_yaw(0)
        
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
                
            if controller:
                controller.reset_head_position()
        
        return "Arm manipulation done"
    
    tool_name_to_override.name = tool_name
    tool_name_to_override.description = tool_description

    return tool_name_to_override


def create_run_robot_policy():
    from core.training_manager import training_manager

    # Build dynamic docstring with available policies
    policies = training_manager.get_policies_for_ai()
    policy_list = ""
    if policies:
        policy_list = "\n\nAvailable policies:\n" + "\n".join(
            f"  - {p['name']}: {p['description'] or 'No description'}"
            for p in policies
        )

    @tool
    def run_robot_policy(policy_name: str, duration_seconds: int = 45) -> str:
        f"""Executes a trained robot policy to perform a physical task using the arms. 
        Use this when the user asks to perform a specific learned skill (e.g. 'pickup_cup', 'wipe_table').
        The policy will take control of the arms and base for the specified duration.{policy_list}
        """
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


def create_scan_doorway():
    @tool
    def scan_doorway() -> str:
        """
        Performs a 'Wiggle Scan' (Left 25 -> Right 50 -> Left 25) to map the doorway ahead.
        Use this when approaching a narrow gap to precisely identify the center.
        If a clear gap is found, the robot will AUTOMATICALLY ALIGN to face the center.
        """
        import time
        import math
        from state import state as robot_state
        
        scanner = robot_state.get_scanner()
        if not scanner:
            return "Error: Scanner system not available."
            
        controller = robot_state.controller
        if not controller:
             return "Error: Robot controller not found."
        
        # Enable Precision Mode for visualization (and save previous)
        previous_mode = robot_state.precision_mode
        robot_state.precision_mode = True
        scanner.clear()
        
        # Scan Parameters
        SWEEP_ANGLE = 25.0
        ROT_SPEED = 0.5 
        DEG_PER_SEC = 30.0 
        DURATION = SWEEP_ANGLE / DEG_PER_SEC
        
        print(f"[TOOL] scan_doorway: Starting scan...")
        
        # Recording Loop
        is_scanning = True
        
        # Shared state for thread synchronization
        scan_state = {
            "phase": "L1",
            "phase_start_time": time.time(),
            "angle": 0.0
        }
        
        def recording_loop():
            while is_scanning:
                t = time.time()
                phase = scan_state["phase"]
                t0 = scan_state["phase_start_time"]
                dt = t - t0
                
                # Estimate current angle
                current_angle = 0.0
                if phase == "L1":
                    current_angle = (dt / DURATION) * SWEEP_ANGLE
                elif phase == "R":
                    current_angle = SWEEP_ANGLE - ((dt / (DURATION*2)) * (SWEEP_ANGLE*2))
                elif phase == "L2":
                     current_angle = -SWEEP_ANGLE + ((dt / DURATION) * SWEEP_ANGLE)
                     
                dist = robot_state.lidar_distance
                if dist is not None:
                     scanner.add_reading(current_angle, dist)
                
                time.sleep(0.04)
        
        # Start Recorder
        recorder = threading.Thread(target=recording_loop, daemon=True)
        recorder.start()
        
        try:
            # 1. Turn Left
            scan_state["phase"] = "L1"
            scan_state["phase_start_time"] = time.time()
            controller.set_velocity_vector(0, 0, ROT_SPEED)
            time.sleep(DURATION)
            
            # 2. Turn Right (Sweep across center)
            scan_state["phase"] = "R"
            scan_state["phase_start_time"] = time.time()
            controller.set_velocity_vector(0, 0, -ROT_SPEED)
            time.sleep(DURATION * 2)
            
            # 3. Turn Left (Return to Center)
            scan_state["phase"] = "L2"
            scan_state["phase_start_time"] = time.time()
            controller.set_velocity_vector(0, 0, ROT_SPEED)
            time.sleep(DURATION)
            
            # Stop
            controller.set_velocity_vector(0, 0, 0)
            
        except Exception as e:
            print(f"Scan interrupted: {e}")
            controller.set_velocity_vector(0, 0, 0)
            return f"Scan failed: {e}"
        finally:
            is_scanning = False
            recorder.join()
            robot_state.precision_mode = previous_mode
            
        # Analysis
        result = scanner.analyze_gap()
        robot_state.last_scan_result = result
        
        if not result['found']:
            return f"Scan Complete. NO GAP FOUND. Reason: {result.get('reason')}. Please reposition and try again."
            
        # Alignment
        center_angle = result['center_angle']
        width = result['width_deg']
        
        align_msg = ""
        if abs(center_angle) > 2.0:
            print(f"[TOOL] Aligning to gap center at {center_angle:.1f} deg")
            align_dur = abs(center_angle) / DEG_PER_SEC
            align_speed = ROT_SPEED if center_angle > 0 else -ROT_SPEED
            
            controller.set_velocity_vector(0, 0, align_speed)
            time.sleep(align_dur)
            controller.set_velocity_vector(0, 0, 0)
            align_msg = f"ALIGNED to center ({center_angle:.1f}°)."
        else:
            align_msg = "ALREADY ALIGNED."
            
        return f"Scan Successful! GAP FOUND. Width: {width:.1f}°. {align_msg} Robot is facing the center. READY FOR BLIND ENTRY."

    return scan_doorway
