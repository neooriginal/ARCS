
"""ARCS Flask Routes"""

import cv2
import time
import numpy as np
import psutil
import subprocess
import os

from typing import Any, Optional, Union
from flask import Blueprint, Response, jsonify, request, render_template, redirect, make_response
from functools import wraps

from state import state
from camera import generate_frames, generate_frames_right, generate_frames_down, generate_frames_fwd
from movement import execute_movement
from arm import arm_controller
import tts
from core.memory_store import memory_store
from core.dataset_recorder import DatasetRecorder
from core.training_manager import training_manager
from core.policy_executor import policy_executor
from core.config_manager import config_manager
from core.auth import (
    hash_password, verify_password, generate_token, verify_token,
    is_auth_configured, require_auth
)

recorder = None 

bp = Blueprint('robot', __name__)


PUBLIC_PATHS = [
    '/login',
    '/api/auth/',
    '/static/',
]


def get_token_from_request() -> Optional[str]:
    token = request.headers.get('Authorization', '').replace('Bearer ', '')
    if not token:
        token = request.cookies.get('auth_token', '')
    return token if token else None


@bp.before_request
def check_auth() -> Optional[Union[Response, tuple[Response, int]]]:
    path = request.path
    
    if any(path.startswith(public) or path == public.rstrip('/') for public in PUBLIC_PATHS):
        return None
    
    if not is_auth_configured():
        if path.startswith('/api/'):
            return jsonify({'error': 'Auth not configured'}), 401
        return redirect('/login')
    
    token = get_token_from_request()
    
    if not token or not verify_token(token):
        if path.startswith('/api/'):
            return jsonify({'error': 'Unauthorized'}), 401
        return redirect('/login')
    
    return None



@bp.route('/login')
def login_page():
    return render_template('login.html')


@bp.route('/api/auth/status')
def auth_status() -> Response:
    configured = is_auth_configured()
    token = get_token_from_request()
    valid = verify_token(token) is not None if token else False
    
    return jsonify({
        'configured': configured,
        'authenticated': valid,
        'username': config_manager.get('AUTH_USERNAME', '')
    })


@bp.route('/api/auth/setup', methods=['POST'])
def auth_setup():
    if is_auth_configured():
        return jsonify({'error': 'Already configured'}), 400
    
    data = request.json or {}
    username = data.get('username', '').strip()
    password = data.get('password', '')
    
    if not username or len(password) < 4:
        return jsonify({'error': 'Username required, password min 4 chars'}), 400
    
    config_manager.set('AUTH_USERNAME', username)
    config_manager.set('AUTH_PASSWORD_HASH', hash_password(password))
    config_manager._save()
    
    token = generate_token(username)
    response = make_response(jsonify({'token': token, 'username': username}))
    response.set_cookie('auth_token', token, max_age=30*24*3600, httponly=True, samesite='Lax')
    return response


@bp.route('/api/auth/login', methods=['POST'])
def auth_login():
    if not is_auth_configured():
        return jsonify({'error': 'Not configured'}), 400
    
    data = request.json or {}
    username = data.get('username', '').strip()
    password = data.get('password', '')
    
    stored_username = config_manager.get('AUTH_USERNAME', '')
    stored_hash = config_manager.get('AUTH_PASSWORD_HASH', '')
    
    if username != stored_username or not verify_password(password, stored_hash):
        return jsonify({'error': 'Invalid credentials'}), 401
    
    token = generate_token(username)
    response = make_response(jsonify({'token': token, 'username': username}))
    response.set_cookie('auth_token', token, max_age=30*24*3600, httponly=True, samesite='Lax')
    return response


@bp.route('/api/auth/password', methods=['POST'])
def auth_change_password() -> Union[Response, tuple[Response, int]]:
    if not is_auth_configured():
        return jsonify({'error': 'Not configured'}), 400
    
    token = get_token_from_request()
    username = verify_token(token) if token else None
    
    if not username:
        return jsonify({'error': 'Unauthorized'}), 401
    
    data = request.json or {}
    current_password = data.get('current_password', '')
    new_password = data.get('new_password', '')
    
    stored_hash = config_manager.get('AUTH_PASSWORD_HASH', '')
    
    if not verify_password(current_password, stored_hash):
        return jsonify({'error': 'Current password incorrect'}), 401
    
    if len(new_password) < 4:
        return jsonify({'error': 'New password too short (min 4 chars)'}), 400
    
    config_manager.set('AUTH_PASSWORD_HASH', hash_password(new_password))
    config_manager._save()
    
    return jsonify({'success': True, 'message': 'Password changed'})


@bp.route('/api/auth/logout', methods=['POST'])
def auth_logout():
    response = make_response(jsonify({'success': True}))
    response.delete_cookie('auth_token')
    return response


@bp.route('/')
def index():
    return render_template('dashboard.html')

@bp.route('/remote')
def remote():
    return render_template('remote.html')

@bp.route('/status')
def get_status():
    return jsonify({
        'controller_connected': state.controller is not None,
        'camera_connected': state.camera is not None and state.camera.isOpened(),
        'arm_connected': state.arm_connected,
        'control_mode': state.get_control_mode(),
        'movement': state.movement,
        'arm_positions': state.get_arm_positions(),
        'error': state.last_error
    })


@bp.route('/api/system/health')
def get_system_health():
    """Get system resource usage and health metrics."""
    try:
        # CPU Usage
        cpu_percent = psutil.cpu_percent(interval=None)
        
        # Memory Usage
        memory = psutil.virtual_memory()
        memory_percent = memory.percent
        memory_used_gb = round(memory.used / (1024**3), 1)
        memory_total_gb = round(memory.total / (1024**3), 1)
        
        # Disk Usage (Root)
        disk = psutil.disk_usage('/')
        disk_percent = disk.percent
        disk_free_gb = round(disk.free / (1024**3), 1)
        
        # Voltage (Mocked for PC, real for some SBCs if available via file reading, but sticking to generic PC stats)
        # If running on battery (like a laptop), we can try to get battery status
        battery = psutil.sensors_battery()
        power_plugged = battery.power_plugged if battery else True
        battery_percent = battery.percent if battery else 100
        
        # Temperature & Throttling (Raspberry Pi / Linux specific)
        temp = 'N/A'
        throttled = None
        
        try:
            # Try psutil first (portable)
            if hasattr(psutil, 'sensors_temperatures'):
                temps = psutil.sensors_temperatures()
                if 'cpu_thermal' in temps:
                    temp = temps['cpu_thermal'][0].current
                elif 'coretemp' in temps:
                    temp = temps['coretemp'][0].current
            
            # Fallback for Raspberry Pi if psutil didn't find it
            if temp == 'N/A' and os.path.exists('/sys/class/thermal/thermal_zone0/temp'):
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                    temp = round(int(f.read()) / 1000.0, 1)
                    
            # Check for Throttling (Raspberry Pi specific)
            # vcgencmd get_throttled
            # 0x50005 = Throttled (under-voltage detected)
            if os.path.exists('/usr/bin/vcgencmd'):
                res = subprocess.check_output(['/usr/bin/vcgencmd', 'get_throttled']).decode('utf-8')
                # Output like: throttled=0x0
                val = int(res.strip().split('=')[1], 16)
                if val != 0:
                    throttled_msg = []
                    if val & 0x1:
                        throttled_msg.append("Under-voltage detected")
                    if val & 0x2:
                        throttled_msg.append("Arm frequency capped")
                    if val & 0x4:
                        throttled_msg.append("Throttled")
                    if val & 0x8:
                        throttled_msg.append("Soft temp limit")
                    throttled = ", ".join(throttled_msg)
            
        except Exception:
            pass

        return jsonify({
            'status': 'ok',
            'cpu': {
                'usage': cpu_percent,
                'temp': temp,
                'status': 'high' if cpu_percent > 80 else 'normal'
            },
            'memory': {
                'percent': memory_percent,
                'used_gb': memory_used_gb,
                'total_gb': memory_total_gb,
                'status': 'high' if memory_percent > 85 else 'normal'
            },
            'disk': {
                'percent': disk_percent,
                'free_gb': disk_free_gb,
                'status': 'high' if disk_percent > 90 else 'normal'
            },
            'power': {
                'battery_percent': battery_percent,
                'plugged': power_plugged,
                'throttled': throttled,
                'status': 'low' if (battery_percent < 20 and not power_plugged) or throttled else 'normal'
            }
        })
    except Exception as e:
        return jsonify({'status': 'error', 'error': str(e)})


@bp.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@bp.route('/video_feed_right')
def video_feed_right():
    return Response(generate_frames_right(), mimetype='multipart/x-mixed-replace; boundary=frame')


@bp.route('/video_feed_down')
def video_feed_down():
    return Response(generate_frames_down(), mimetype='multipart/x-mixed-replace; boundary=frame')


@bp.route('/video_feed_fwd')
def video_feed_fwd():
    return Response(generate_frames_fwd(), mimetype='multipart/x-mixed-replace; boundary=frame')


@bp.route('/api/logs')
def get_logs():
    """Get system logs, optionally filtered by timestamp."""
    since = float(request.args.get('since', 0))
    if state.log_handler:
        return jsonify({'logs': state.log_handler.get_logs(since)})
    return jsonify({'logs': []})





@bp.route('/move', methods=['POST'])
def move():
    if state.controller is None:
        return jsonify({'status': 'error', 'error': 'No controller'})
    
    data = request.json
    state.update_movement(data)
    
    movement = state.get_movement()
    success = execute_movement(movement)
    
    if any(movement.values()):
        state.last_remote_activity = time.time()
        state.last_movement_activity = time.time()

    
    return jsonify({'status': 'ok' if success else 'error'})


# Mode routes

@bp.route('/mode', methods=['GET'])
def get_mode():
    return jsonify({'mode': state.get_control_mode()})


@bp.route('/mode', methods=['POST'])
def set_mode():
    data = request.json
    mode = data.get('mode', 'drive')
    
    if state.set_control_mode(mode):
        return jsonify({'status': 'ok', 'mode': mode})
    return jsonify({'status': 'error', 'error': f'Invalid mode: {mode}'})


# Arm routes

@bp.route('/arm_position')
def get_arm_position():
    if not state.arm_connected:
        return jsonify({'error': 'Arm not connected'})
    
    try:
        pos = state.controller.get_arm_position()
        state.update_arm_positions(pos)
        return jsonify({'positions': pos})
    except Exception as e:
        return jsonify({'error': str(e)})


@bp.route('/arm', methods=['POST'])
def set_arm():
    if not state.arm_connected:
        return jsonify({'status': 'error', 'error': 'Arm not connected'})
    
    data = request.json
    positions = data.get('positions', {})
    
    try:
        result = state.controller.set_arm_position(positions)
        state.update_arm_positions(result)
        return jsonify({'status': 'ok', 'positions': result})
    except Exception as e:
        return jsonify({'status': 'error', 'error': str(e)})


@bp.route('/arm/mouse', methods=['POST'])
def arm_mouse():
    if not state.arm_connected:
        return jsonify({'status': 'error', 'error': 'Arm not connected'})
    
    data = request.json
    delta_x = float(data.get('deltaX', 0))
    delta_y = float(data.get('deltaY', 0))
    
    try:
        targets = arm_controller.handle_mouse_move(delta_x, delta_y)
        result = state.controller.set_arm_position(targets)
        state.update_arm_positions(result)
        return jsonify({'status': 'ok', 'positions': result})
    except Exception as e:
        return jsonify({'status': 'error', 'error': str(e)})


@bp.route('/arm/scroll', methods=['POST'])
def arm_scroll():
    if not state.arm_connected:
        return jsonify({'status': 'error', 'error': 'Arm not connected'})
    
    data = request.json
    delta = float(data.get('delta', 0))
    
    try:
        arm_controller.handle_scroll(delta)
        targets = arm_controller.get_targets()
        result = state.controller.set_arm_position(targets)
        state.update_arm_positions(result)
        return jsonify({'status': 'ok', 'wrist_roll': result.get('wrist_roll', 0)})
    except Exception as e:
        return jsonify({'status': 'error', 'error': str(e)})


@bp.route('/arm/key', methods=['POST'])
def arm_key():
    if not state.arm_connected:
        return jsonify({'status': 'error', 'error': 'Arm not connected'})
    
    data = request.json
    key = data.get('key', '')
    
    try:
        if key == 'q':
            arm_controller.handle_shoulder_pan(-1)
        elif key == 'e':
            arm_controller.handle_shoulder_pan(1)
        elif key == 'r':
            arm_controller.handle_wrist_flex(1)
        elif key == 'f':
            arm_controller.handle_wrist_flex(-1)
        elif key == 't':
            arm_controller.handle_elbow_flex(-1)
        elif key == 'g':
            arm_controller.handle_elbow_flex(1)
        
        targets = arm_controller.get_targets()
        result = state.controller.set_arm_position(targets)
        state.update_arm_positions(result)
        return jsonify({'status': 'ok', 'positions': result})
    except Exception as e:
        return jsonify({'status': 'error', 'error': str(e)})


@bp.route('/gripper', methods=['POST'])
def set_gripper():
    if not state.arm_connected:
        return jsonify({'status': 'error', 'error': 'Arm not connected'})
    
    data = request.json
    closed = bool(data.get('closed', False))
    
    try:
        arm_controller.set_gripper(closed)
        result = state.controller.set_gripper(closed)
        state.gripper_closed = closed
        return jsonify({'status': 'ok', 'closed': closed, 'angle': result})
    except Exception as e:
        return jsonify({'status': 'error', 'error': str(e)})


# AI Routes

@bp.route('/ai/start', methods=['POST'])
def ai_start():
    if not state.agent:
        return jsonify({'status': 'error', 'error': 'AI Agent not initialized'})

    current_task = None
    if hasattr(state.agent, 'current_task'):
        current_task = state.agent.current_task

    # Clear previous context
    state.agent.reset()
    
    # Restore task
    if current_task:
        state.agent.set_task(current_task)
    
    # Reset wheel speed to default when AI starts
    state.reset_wheel_speed()
    
    state.ai_enabled = True
    state.add_ai_log("AI Started")
    return jsonify({'status': 'ok'})

@bp.route('/ai/stop', methods=['POST'])
def ai_stop():
    state.ai_enabled = False
    state.precision_mode = False
    
    # Clear context on stop as well
    if state.agent:
        state.agent.reset()
        
    state.add_ai_log("AI Stopped")
    return jsonify({'status': 'ok'})

@bp.route('/ai/task', methods=['POST'])
def ai_task():
    if not state.agent:
        return jsonify({'status': 'error', 'error': 'AI Agent not initialized'})
    data = request.json
    task = data.get('task', '')
    if task:
        state.agent.set_task(task)
        state.add_ai_log(f"New Task: {task}")
    return jsonify({'status': 'ok'})

@bp.route('/ai/status')
def ai_status():
    return jsonify({
        'enabled': state.ai_enabled,
        'status': state.ai_status,
        'logs': state.ai_logs
    })

@bp.route('/emergency_stop', methods=['POST'])
def emergency_stop():
    state.ai_enabled = False
    state.precision_mode = False
    state.stop_all_movement()
    policy_executor.stop_execution()
    if state.robot_system:
        state.robot_system.emergency_stop()
    state.add_ai_log("EMERGENCY STOP TRIGGERED")
    return jsonify({'status': 'ok'})

@bp.route('/ai')
def ai_page():
    return render_template('ai_control.html')


# TTS Routes

@bp.route('/tts/speak', methods=['POST'])
def tts_speak():
    data = request.json
    text = data.get('text', '')
    if text:
        tts.speak(text)
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error', 'error': 'No text provided'}), 400


# Actuator Routes

@bp.route('/actuator/move', methods=['POST'])
def actuator_move():
    if not state.actuator_connected or not state.actuator:
        return jsonify({'status': 'error', 'error': 'Actuator not connected'}), 400

    data = request.json or {}
    direction = data.get('direction', 'stop')
    speed = int(data.get('speed', 100))

    try:
        if direction == 'extend':
            state.actuator.extend(speed)
        elif direction == 'retract':
            state.actuator.retract(speed)
        else:
            state.actuator.stop()
        return jsonify({'status': 'ok', 'direction': direction, 'speed': speed})
    except Exception as e:
        return jsonify({'status': 'error', 'error': str(e)}), 500


@bp.route('/actuator/status')
def actuator_status():
    connected = state.actuator_connected and state.actuator is not None
    telemetry = state.actuator.get_telemetry() if connected else {}
    return jsonify({'connected': connected, 'telemetry': telemetry})


# BMS Routes

@bp.route('/api/bms/status')
def bms_status():
    return jsonify({
        'connected': state.bms_connected,
        'soc': state.bms_soc,
        'voltage': state.bms_voltage,
        'current': state.bms_current,
        'temp': state.bms_temp,
        'temps': state.bms_temps,
        'cells': state.bms_cells,
        'cycles': state.bms_cycles,
        'remain_cap': state.bms_remain_cap,
        'charging': state.bms_charging,
        'discharging': state.bms_discharging,
        'error': state.bms_error,
        'last_update': state.bms_last_update,
    })


@bp.route('/api/bms/charging', methods=['POST'])
def bms_set_charging():
    if not state.bms_connected or not state.bms_client:
        return jsonify({'status': 'error', 'error': 'BMS not connected'}), 400

    data = request.json or {}
    enabled = bool(data.get('enabled', True))

    try:
        ok = state.bms_client.set_charging(enabled)
        if ok:
            state.bms_charging = enabled
            return jsonify({'status': 'ok', 'charging': enabled})
        return jsonify({'status': 'error', 'error': 'Command failed'}), 500
    except Exception as e:
        return jsonify({'status': 'error', 'error': str(e)}), 500


@bp.route('/api/bms/scan', methods=['POST'])
def bms_scan():
    import asyncio
    try:
        from bms import scan_bms_devices
        loop = asyncio.new_event_loop()
        devices = loop.run_until_complete(scan_bms_devices(timeout=5.0))
        loop.close()
        return jsonify({'devices': devices})
    except Exception as e:
        return jsonify({'devices': [], 'error': str(e)}), 500

# Wheel Speed Routes

@bp.route('/wheels/speed', methods=['POST'])
def set_wheel_speed():
    data = request.json
    speed = data.get('speed', 10000)
    try:
        state.set_wheel_speed(speed)
        return jsonify({'status': 'ok', 'speed': state.get_wheel_speed()})
    except Exception as e:
        return jsonify({'status': 'error', 'error': str(e)}), 400

@bp.route('/wheels/speed', methods=['GET'])
def get_wheel_speed():
    return jsonify({'speed': state.get_wheel_speed()})


@bp.route('/display')
def display_page():
    """Serve the fullscreen display visualization page."""
    return render_template('display.html')


@bp.route('/display/state')
def display_state():
    """API endpoint for display to poll current robot state."""
    # Determine control mode: remote, ai, or idle
    is_remote = (time.time() - state.last_remote_activity) < 3  # Active within 3 seconds
    if state.ai_enabled:
        control_mode = 'ai'
    elif is_remote:
        control_mode = 'remote'
    else:
        control_mode = 'idle'
    
    return jsonify({
        'ai_enabled': state.ai_enabled,
        'ai_status': state.ai_status,
        'current_task': state.agent.current_task if state.agent and hasattr(state.agent, 'current_task') else None,
        'controller_connected': state.controller is not None,
        
        # Detailed Motor Status
        'wheels_connected': state.controller is not None,
        'arm_connected': state.arm_connected,
        
        # Camera Status
        'camera_connected': state.camera is not None and state.camera.isOpened() if state.camera else False,
        'camera_main_connected': state.camera is not None and state.camera.isOpened() if state.camera else False,
        'camera_right_connected': state.camera_right is not None and state.camera_right.isOpened() if state.camera_right else False,
        'camera_down_connected': state.camera_down is not None and state.camera_down.isOpened() if state.camera_down else False,
        'camera_fwd_connected': state.camera_fwd is not None and state.camera_fwd.isOpened() if state.camera_fwd else False,

        # Actuator Status
        'actuator_connected': state.actuator_connected,
        
        # Lidar Status
        'lidar_connected': state.lidar360 is not None and state.lidar360.connected if state.lidar360 else False,
        'lidar_distance': state.lidar_distance,
        'lidar_scan': state.lidar360.get_decimated_scan(10) if state.lidar360 else None,
        
        'control_mode': control_mode,
        'precision_mode': state.precision_mode,
        'blockage': state.get_detector().latest_blockage if state.detector else {}
    })





def generate_cv_frames():
    """Generate CV-processed frames showing what the AI sees with Obstacle Detection."""
    detector = state.get_detector()
    
    # Pre-generate fallback frames
    STREAM_WIDTH = 640
    STREAM_HEIGHT = 480
    
    error_frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), np.uint8)
    cv2.putText(error_frame, "AI VISION: NO SIGNAL", (20, STREAM_HEIGHT//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    _, buffer_error = cv2.imencode('.jpg', error_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    error_bytes = buffer_error.tobytes()

    # Yield initial frame immediately to ensure headers are sent
    yield (b'--frame\r\n'
           b'Content-Type: image/jpeg\r\n\r\n' + error_bytes + b'\r\n')

    while state.running:
        if state.robot_system is None:
            time.sleep(1)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + error_bytes + b'\r\n')
            continue
        
        try:
            # Thread-safe frame capture
            frame = state.robot_system.get_frame()
            if frame is None:
                time.sleep(0.1)
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + error_bytes + b'\r\n')
                continue
            
            # Process frame using the new ObstacleDetector
            # Now returns (safe_actions, overlay, metrics)
            safe_actions, overlay, metrics = detector.process(frame)
            
            # Add AI status overlay on top of the detector's overlay
            if state.ai_enabled:
                cv2.putText(overlay, "AI: ACTIVE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(overlay, "AI: PAUSED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
            
            # Add task
            if state.agent and hasattr(state.agent, 'current_task'):
                task_text = state.agent.current_task[:40] if len(state.agent.current_task) > 40 else state.agent.current_task
                cv2.putText(overlay, task_text, (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Display Safe Actions cleanly
            if "FORWARD" not in safe_actions:
                 cv2.putText(overlay, "BLOCKED AHEAD", (overlay.shape[1]//2 - 80, overlay.shape[0]//2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            _, buffer = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 70])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        except Exception as e:
            time.sleep(0.05)


@bp.route('/ai_video_feed')
def ai_video_feed():
    return Response(generate_cv_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@bp.route('/memory')
def memory_page():
    return render_template('memory.html')


@bp.route('/api/memory', methods=['GET'])
def get_memories():
    notes = memory_store.get_notes(limit=50)
    return jsonify({'notes': notes})


@bp.route('/api/memory', methods=['POST'])
def add_memory():
    data = request.json
    category = data.get('category', 'other')
    content = data.get('content', '')
    if not content:
        return jsonify({'status': 'error', 'error': 'Content required'}), 400
    note_id = memory_store.save_note(category, content)
    return jsonify({'status': 'ok', 'id': note_id})


@bp.route('/api/memory/<int:note_id>', methods=['DELETE'])
def delete_memory(note_id):
    with memory_store._db_lock:
        cursor = memory_store.conn.cursor()
        cursor.execute('DELETE FROM notes WHERE id = ?', (note_id,))
        memory_store.conn.commit()
        deleted = cursor.rowcount > 0
    if deleted:
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error', 'error': 'Note not found'}), 404


@bp.route('/api/memory/<int:note_id>', methods=['PUT'])
def update_memory(note_id):
    from core.memory_store import memory_store
    data = request.json
    category = data.get('category')
    content = data.get('content')
    
    with memory_store._db_lock:
        cursor = memory_store.conn.cursor()
        if category and content:
            cursor.execute('UPDATE notes SET category = ?, content = ? WHERE id = ?', (category, content, note_id))
        elif content:
            cursor.execute('UPDATE notes SET content = ? WHERE id = ?', (content, note_id))
        elif category:
            cursor.execute('UPDATE notes SET category = ? WHERE id = ?', (category, note_id))
        memory_store.conn.commit()
        updated = cursor.rowcount > 0
    
    if updated:
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error', 'error': 'Note not found'}), 404


@bp.route('/api/memory/clear', methods=['POST'])
def clear_memories():
    memory_store.clear_all()
    return jsonify({'status': 'ok'})





# VR Control Routes

@bp.route('/vr')
def vr_page():
    """VR control page for Quest 3."""
    return render_template('vr_control.html')


@bp.route('/api/vr/status')
def vr_status():
    """Get VR server status."""
    from vr_arm_controller import vr_arm_controller
    
    connected = False
    arm_mode = 'idle'
    
    if vr_arm_controller:
        if vr_arm_controller.vr_server:
            connected = vr_arm_controller.vr_server.is_running
        arm_mode = vr_arm_controller.mode.value if vr_arm_controller.mode else 'idle'
    
    return jsonify({
        'server_running': connected,
        'arm_mode': arm_mode,
        'arm_connected': state.arm_connected
    })

# Recording Routes

@bp.route('/api/recording/start', methods=['POST'])
def start_recording():
    global recorder
    data = request.json
    dataset_name = data.get('dataset_name')
    
    if not dataset_name:
        return jsonify({'status': 'error', 'error': 'Dataset name required'}), 400

    # Enforce HF Login for Recording
    if not training_manager.get_hf_user():
         return jsonify({'status': 'error', 'error': 'HuggingFace Login Required'}), 403

    # Lazy init recorder with current state cameras
    if recorder is None:
        recorder = DatasetRecorder(main_camera=state.camera)
        if state.camera_right is not None:
            recorder.right_camera = state.camera_right

    if recorder.start_recording(dataset_name):
        return jsonify({'status': 'ok', 'dataset_name': dataset_name})
    else:
        return jsonify({'status': 'error', 'error': 'Recording already in progress or failed'}), 409

@bp.route('/api/recording/stop', methods=['POST'])
def stop_recording():
    global recorder
    if recorder and recorder.stop_recording():
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error', 'error': 'Not recording'}), 400

@bp.route('/api/recording/status')
def recording_status():
    global recorder
    is_recording = recorder.is_recording if recorder else False
    return jsonify({
        'is_recording': is_recording,
        'dataset_name': recorder.dataset_name if recorder else None,
        'episode_idx': recorder.episode_idx if recorder else 0,
        'frame_count': recorder.frame_idx if recorder and is_recording else 0
    })


# Training Routes

@bp.route('/training')
def training_page():
    return render_template('training.html')

@bp.route('/settings')
def settings_page():
    return render_template('settings.html')

@bp.route('/api/training/datasets')
def list_datasets():
    datasets = training_manager.list_datasets()
    return jsonify({'datasets': datasets})

@bp.route('/api/training/policies')
def list_training_policies():
    policies = training_manager.list_policies()
    return jsonify({'policies': policies})

@bp.route('/api/training/start', methods=['POST'])
def start_training():
    data = request.json
    dataset = data.get('dataset')
    job_name = data.get('job_name')
    device = data.get('device', 'auto') 
    steps = int(data.get('steps', 2000))
    
    if not dataset:
        return jsonify({'status': 'error', 'error': 'Dataset required'}), 400
        
    if not job_name:
        return jsonify({'status': 'error', 'error': 'Job Name required'}), 400

    if device == 'remote':
         success, msg = training_manager.queue_remote_training(dataset, job_name, steps)
    else:
         success, msg = training_manager.start_training(dataset, job_name, device, steps)
         
    if success:
        return jsonify({'status': 'ok', 'job_name': msg})
    else:
        return jsonify({'status': 'error', 'error': msg}), 400

@bp.route('/api/training/stop', methods=['POST'])
def stop_training_job():
    if training_manager.stop_training():
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error', 'error': 'No active training'}), 400

# ---- Worker Routes ----
@bp.route('/api/worker/heartbeat', methods=['POST'])
def worker_heartbeat():
    return jsonify(training_manager.register_worker_heartbeat(request.json))

@bp.route('/api/worker/update', methods=['POST'])
def worker_update():
    # Update status (e.g. working / offline)
    training_manager.register_worker_heartbeat(request.json)
    return jsonify({'status': 'ok'})

@bp.route('/api/worker/log', methods=['POST'])
def worker_log():
    training_manager.remote_log(request.json)
    return jsonify({'status': 'ok'})

@bp.route('/api/worker/complete', methods=['POST'])
def worker_complete():
    training_manager.remote_complete(request.json)
    return jsonify({'status': 'ok'})

@bp.route('/api/training/worker_status', methods=['GET'])
def get_worker_status():
    return jsonify({'workers': training_manager.get_worker_status()})


@bp.route('/api/training/status')
def training_status():
    since = int(request.args.get('since', 0))
    status = training_manager.get_status()
    logs = training_manager.get_logs(since=since)
    return jsonify({
        'status': status,
        'logs': logs
    })

@bp.route('/api/training/datasets/delete', methods=['POST'])
def delete_dataset_route():
    data = request.json
    dataset_name = data.get('dataset_name')
    if not dataset_name:
        return jsonify({'status': 'error', 'error': 'Dataset Name required'}), 400
    
    success, msg = training_manager.delete_dataset(dataset_name)
    return jsonify({'status': 'ok' if success else 'error', 'message': msg})

@bp.route('/api/auth/hf/status', methods=['GET'])
def get_hf_status_check():
    user = training_manager.get_hf_user()
    return jsonify({
        'logged_in': bool(user),
        'username': user
    })

@bp.route('/api/training/datasets/rename', methods=['POST'])
def rename_dataset_route():
    data = request.json
    old_name = data.get('old_name')
    new_name = data.get('new_name')
    
    if not old_name or not new_name:
        return jsonify({'status': 'error', 'error': 'Old and New names required'}), 400
        
    success, msg = training_manager.rename_dataset(old_name, new_name)
    return jsonify({'status': 'ok' if success else 'error', 'message': msg})

@bp.route('/api/training/policies/rename', methods=['POST'])
def rename_policy_route():
    data = request.json
    old_name = data.get('old_name')
    new_name = data.get('new_name')
    
    if not old_name or not new_name:
        return jsonify({'status': 'error', 'error': 'Old and New names required'}), 400
        
    success, msg = training_manager.rename_policy(old_name, new_name)
    return jsonify({'status': 'ok' if success else 'error', 'message': msg})

@bp.route('/api/training/policies/delete', methods=['POST'])
def delete_policy_route():
    data = request.json
    policy_name = data.get('policy_name')
    
    if not policy_name:
        return jsonify({'status': 'error', 'error': 'Policy Name required'}), 400
        
    success, msg = training_manager.delete_policy(policy_name)
    return jsonify({'status': 'ok' if success else 'error', 'message': msg})

@bp.route('/api/training/auth', methods=['GET'])
def get_hf_auth_status():
    user = training_manager.get_hf_user()
    return jsonify({'username': user})

@bp.route('/api/training/auth/login', methods=['POST'])
def hf_login():
    data = request.json
    token = data.get('token')
    if not token:
        return jsonify({'status': 'error', 'error': 'Token required'}), 400
    
    success, msg = training_manager.hf_login(token)
    return jsonify({'status': 'ok' if success else 'error', 'message': msg})

@bp.route('/api/training/auth/logout', methods=['POST'])
def hf_logout():
    success, msg = training_manager.hf_logout()
    return jsonify({'status': 'ok' if success else 'error', 'message': msg})


# Policy Execution Routes

@bp.route('/api/policies/load', methods=['POST'])
def load_policy():
    data = request.json
    policy_name = data.get('policy_name')
    device = data.get('device', 'cuda')
    
    if not policy_name:
        return jsonify({'status': 'error', 'error': 'Policy Name required'}), 400
        
    if policy_executor.load_policy(policy_name, device):
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error', 'error': 'Failed to load policy'}), 500

@bp.route('/api/policies/run', methods=['POST'])
def run_policy():
    if policy_executor.start_execution():
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error', 'error': 'Failed to start or already running'}), 500

@bp.route('/api/policies/stop', methods=['POST'])
def stop_policy():
    if policy_executor.stop_execution():
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error', 'error': 'Not running'}), 400

@bp.route('/api/policies/status')
def policy_status():
    return jsonify({
        'is_running': policy_executor.is_running,
        'current_policy': policy_executor.current_policy_name
    })

@bp.route('/api/training/policies/delete', methods=['POST'])
def delete_policy():
    data = request.json
    policy_name = data.get('policy_name')
    if not policy_name:
        return jsonify({'status': 'error', 'error': 'Policy name required'}), 400
        
    success, message = training_manager.delete_policy(policy_name)
    if success:
        return jsonify({'status': 'ok', 'message': message})
    else:
        return jsonify({'status': 'error', 'error': message}), 500

@bp.route('/api/policies/update', methods=['POST'])
def update_policy_info():
    data = request.json
    policy_name = data.get('name')
    if not policy_name:
        return jsonify({'status': 'error', 'error': 'Policy Name required'}), 400
        
    # extract known metadata fields
    update_data = {}
    if 'enabled' in data:
        update_data['enabled'] = bool(data['enabled'])
    if 'description' in data:
        update_data['description'] = str(data['description']).strip()
        
    if training_manager.update_policy_metadata(policy_name, update_data):
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error', 'error': 'Failed to update metadata'}), 500


# ============ Configuration API ============

@bp.route('/api/config', methods=['GET'])
def get_config():
    """Return all configuration values."""
    current_config = config_manager.get_all()
    
    # Mask API Key
    api_key = current_config.get('OPENAI_API_KEY', '')
    if len(api_key) > 10:
        current_config['OPENAI_API_KEY'] = 'sk-' + '*' * (len(api_key) - 3)
        
    return jsonify({
        'config': current_config,
        'defaults': config_manager.get_defaults()
    })

@bp.route('/api/config', methods=['POST'])
def save_config():
    """Save configuration values."""
    data = request.json
    if not data:
        return jsonify({'status': 'error', 'error': 'No data'}), 400
    
    # Don't save if it's the mask
    new_key = data.get('OPENAI_API_KEY', '')
    if new_key.startswith('sk-***'):
        del data['OPENAI_API_KEY']
    
    if config_manager.update(data):
        return jsonify({'status': 'ok'})
    return jsonify({'status': 'error', 'error': 'Failed to save'}), 500

@bp.route('/api/ports', methods=['GET'])
def list_ports():
    """List available serial and video ports."""
    import platform
    import glob
    
    result = {'serial': [], 'video': []}
    
    # Serial ports
    try:
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            result['serial'].append({
                'device': port.device,
                'description': port.description or port.device
            })
    except Exception:
        pass
    
    # Video devices
    system = platform.system()
    if system == 'Linux':
        for dev in sorted(glob.glob('/dev/video*')):
            result['video'].append({'device': dev, 'description': dev})
    elif system == 'Windows':
        for i in range(5):
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if cap.isOpened():
                result['video'].append({'device': str(i), 'description': f'Camera {i}'})
                cap.release()
    else:
        for i in range(5):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                result['video'].append({'device': str(i), 'description': f'Camera {i}'})
                cap.release()
    
    return jsonify(result)


