"""
RoboCrew Control System - Camera Module
Handles camera initialization and MJPEG streaming.
"""

import time
import cv2
import numpy as np

from config import (
    CAMERA_PORT, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_BUFFER_SIZE,
    STREAM_WIDTH, STREAM_HEIGHT, STREAM_JPEG_QUALITY
)
from state import state


import threading

def init_camera():
    print(f"📷 Connecting camera ({CAMERA_PORT})...", end=" ", flush=True)
    try:
        if state.camera is not None and state.camera.isOpened():
            print("✓ (Already open)")
            return True

        camera = cv2.VideoCapture(CAMERA_PORT)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        
        if camera.isOpened():
            print("✓")
            state.camera = camera
            
            # Start background capture thread
            capture_thread = threading.Thread(target=_capture_loop, daemon=True)
            capture_thread.start()
            
            return True
        else:
            print("⚠ Warning: Camera may not be available")
            state.last_error = "Camera not available"
            state.camera = None
            return False
    except Exception as e:
        print(f"✗ Failed: {e}")
        state.camera = None
        state.last_error = f"Camera init failed: {e}"
        return False


def _capture_loop():
    print("[Camera] Capture thread started")
    while state.running and state.camera and state.camera.isOpened():
        try:
            ret, frame = state.camera.read()
            if ret:
                state.latest_frame = frame
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[Camera] Thread error: {e}")
            time.sleep(0.1)
    print("[Camera] Capture thread stopped")


def generate_frames():
    blank_frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), np.uint8)
    cv2.putText(blank_frame, "WAITING...", (20, STREAM_HEIGHT//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    _, blank_buffer = cv2.imencode('.jpg', blank_frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
    blank_bytes = blank_buffer.tobytes()

    while state.running:
        if not hasattr(state, 'latest_frame') or state.latest_frame is None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + blank_bytes + b'\r\n')
            time.sleep(0.5)
            continue
        
        try:
            frame = state.latest_frame
            stream_frame = cv2.resize(frame, (STREAM_WIDTH, STREAM_HEIGHT), interpolation=cv2.INTER_NEAREST)

            _, buffer = cv2.imencode('.jpg', stream_frame, [
                cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY,
                cv2.IMWRITE_JPEG_OPTIMIZE, 0
            ])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            
            time.sleep(0.03)
            
        except Exception as e:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + blank_bytes + b'\r\n')
            time.sleep(0.1)


def release_camera():
    """Release camera resources."""
    if state.camera:
        try:
            state.camera.release()
            print("✓ Camera released")
        except Exception as e:
            print(f"✗ Camera cleanup error: {e}")
    state.camera = None
