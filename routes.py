"""
RoboCrew Control System - Flask Routes
Blueprint containing all HTTP endpoints.
"""

from flask import Blueprint, Response, jsonify, request, render_template

from state import state
from camera import generate_frames
from movement import execute_movement

# Create blueprint
bp = Blueprint('robot', __name__)


@bp.route('/')
def index():
    """Serve the main control interface."""
    return render_template('index.html')


@bp.route('/status')
def get_status():
    """Get connection status for debugging."""
    return jsonify({
        'controller_connected': state.controller is not None,
        'camera_connected': state.camera is not None and state.camera.isOpened(),
        'head_yaw': state.head_yaw,
        'head_pitch': state.head_pitch,
        'movement': state.movement,
        'error': state.last_error
    })


@bp.route('/video_feed')
def video_feed():
    """MJPEG video stream endpoint."""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@bp.route('/head_position')
def get_head_position():
    """Get current head servo positions - read fresh from servos."""
    if state.controller is None:
        return jsonify({'error': 'No controller connected'})
    
    try:
        pos = state.controller.get_head_position()
        print(f"[HEAD READ] Raw position from servos: {pos}")
        yaw = round(pos.get(7, 0), 1)
        pitch = round(pos.get(8, 0), 1)
        print(f"[HEAD READ] Parsed: yaw={yaw}, pitch={pitch}")
        
        # Update cached values
        state.head_yaw = yaw
        state.head_pitch = pitch
        return jsonify({'yaw': yaw, 'pitch': pitch})
    except Exception as e:
        state.last_error = f"Head read error: {str(e)}"
        print(f"[HEAD READ ERROR] {e}")
        return jsonify({'error': str(e)})


@bp.route('/head', methods=['POST'])
def set_head():
    """Set head yaw and pitch - smooth incremental control."""
    if state.controller is None:
        return jsonify({'status': 'error', 'error': 'No controller connected'})
    
    data = request.json
    yaw = float(data.get('yaw', state.head_yaw))
    pitch = float(data.get('pitch', state.head_pitch))
    
    print(f"[HEAD WRITE] Commanding: yaw={yaw}, pitch={pitch}")
    
    try:
        state.controller.turn_head_yaw(yaw)
        state.controller.turn_head_pitch(pitch)
        state.head_yaw = yaw
        state.head_pitch = pitch
        return jsonify({'status': 'ok', 'yaw': yaw, 'pitch': pitch})
    except Exception as e:
        state.last_error = f"Head write error: {str(e)}"
        print(f"[HEAD WRITE ERROR] {e}")
        return jsonify({'status': 'error', 'error': str(e)})


@bp.route('/move', methods=['POST'])
def move():
    """Update movement state from WASD keys."""
    if state.controller is None:
        return jsonify({'status': 'error', 'error': 'No controller connected'})
    
    data = request.json
    state.update_movement(data)
    
    # Execute movement immediately for responsiveness
    movement = state.get_movement()
    success = execute_movement(movement)
    
    if success:
        return jsonify({'status': 'ok'})
    else:
        return jsonify({'status': 'error', 'error': state.last_error})
