"""
RoboCrew Control System - Camera Module
Handles camera initialization and MJPEG streaming.
"""

import time
import cv2
import numpy as np
import threading

from config import (
    CAMERA_PORT, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_BUFFER_SIZE,
    STREAM_WIDTH, STREAM_HEIGHT, STREAM_JPEG_QUALITY,
    CAMERA_RIGHT_PORT
)
from state import state



def init_camera():
    print(f"📷 Connecting camera ({CAMERA_PORT})...", end=" ", flush=True)
    try:
        if state.camera is not None and state.camera.isOpened():
            print("✓ (Already open)")
            return True

        # Use V4L2 backend
        try:
            camera = cv2.VideoCapture(CAMERA_PORT, cv2.CAP_V4L2)
        except Exception:
            # Fallback to default
            camera = cv2.VideoCapture(CAMERA_PORT)
            
        # Request MJPEG format
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        camera.set(cv2.CAP_PROP_FPS, 30)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if camera.isOpened():
            # Drain stale frames
            for _ in range(5):
                camera.grab()
            
            print("✓")
            state.camera = camera
            
            # Start background capture thread
            threading.Thread(target=_capture_loop, daemon=True).start()
        else:
            print("✗")
            state.camera = None
            return False

        # Initialize Right Camera (Optional)
        print(f"📷 Connecting right camera ({CAMERA_RIGHT_PORT})...", end=" ", flush=True)
        try:
            if state.camera_right is not None and state.camera_right.isOpened():
                print("✓ (Already open)")
            else:
                camera_right = cv2.VideoCapture(CAMERA_RIGHT_PORT, cv2.CAP_V4L2)
                
                # Request MJPEG format
                camera_right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                camera_right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
                camera_right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
                camera_right.set(cv2.CAP_PROP_FPS, 30)
                camera_right.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                if camera_right.isOpened():
                    for _ in range(5):
                        camera_right.grab()
                    print("✓")
                    state.camera_right = camera_right
                    threading.Thread(target=_capture_loop_right, daemon=True).start()
                else:
                    print(f"⚠ (Not found)")
                    state.camera_right = None
        except Exception:
            print(f"⚠ (Failed)")
            state.camera_right = None
            
        return True

    except Exception as e:
        print(f"✗ Failed: {e}")
        state.camera = None
        state.last_error = f"Camera init failed: {e}"
        return False


# Frame synchronization
frame_condition = threading.Condition()
encoded_frame = None
current_frame_id = 0

frame_condition_right = threading.Condition()
encoded_frame_right = None
current_frame_id_right = 0


def _capture_loop():
    global encoded_frame, current_frame_id
    print("[Camera] Capture thread started")
    
    # Pre-allocate blank frame for fallback
    blank_frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), np.uint8)
    cv2.putText(blank_frame, "WAITING...", (20, STREAM_HEIGHT//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    _, blank_buffer = cv2.imencode('.jpg', blank_frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
    blank_bytes = blank_buffer.tobytes()
    
    with frame_condition:
        encoded_frame = blank_bytes
        frame_condition.notify_all()

    while state.running and state.camera and state.camera.isOpened():
        try:
            # Non-blocking capture: grab() + retrieve() instead of read()
            grabbed = state.camera.grab()
            if grabbed:
                ret, frame = state.camera.retrieve()
            else:
                ret = False
                frame = None
            
            if ret and frame is not None:
                state.latest_frame = frame
                state.frame_id += 1
                
                # Resize and encode for streaming
                stream_frame = cv2.resize(frame, (STREAM_WIDTH, STREAM_HEIGHT), interpolation=cv2.INTER_LINEAR)
                _, buffer = cv2.imencode('.jpg', stream_frame, [
                    cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY,
                    cv2.IMWRITE_JPEG_OPTIMIZE, 0
                ])
                
                with frame_condition:
                    encoded_frame = buffer.tobytes()
                    current_frame_id = state.frame_id
                    frame_condition.notify_all()
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[Camera] Thread error: {e}")
            time.sleep(0.1)
    print("[Camera] Capture thread stopped")


def _capture_loop_right():
    global encoded_frame_right, current_frame_id_right
    print("[Camera] Right capture thread started")
    
    # Pre-allocate blank frame
    blank_frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), np.uint8)
    cv2.putText(blank_frame, "WAITING...", (20, STREAM_HEIGHT//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    _, blank_buffer = cv2.imencode('.jpg', blank_frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
    blank_bytes = blank_buffer.tobytes()
    
    with frame_condition_right:
        encoded_frame_right = blank_bytes
        frame_condition_right.notify_all()

    while state.running and state.camera_right and state.camera_right.isOpened():
        try:
            # Non-blocking capture
            grabbed = state.camera_right.grab()
            if grabbed:
                ret, frame = state.camera_right.retrieve()
            else:
                ret = False
                frame = None
            
            if ret and frame is not None:
                state.latest_frame_right = frame
                state.frame_id_right += 1
                
                # Resize and encode
                stream_frame = cv2.resize(frame, (STREAM_WIDTH, STREAM_HEIGHT), interpolation=cv2.INTER_LINEAR)
                _, buffer = cv2.imencode('.jpg', stream_frame, [
                    cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY,
                    cv2.IMWRITE_JPEG_OPTIMIZE, 0
                ])
                
                with frame_condition_right:
                    encoded_frame_right = buffer.tobytes()
                    current_frame_id_right = state.frame_id_right
                    frame_condition_right.notify_all()
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[Camera] Right thread error: {e}")
            time.sleep(0.1)
    print("[Camera] Right capture thread stopped")


def generate_frames():
    global encoded_frame, current_frame_id
    
    current_frame = encoded_frame
    last_sent_frame_id = 0
    
    # Create fallback frames
    blank_frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), np.uint8)
    cv2.putText(blank_frame, "STARTING...", (20, STREAM_HEIGHT//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    _, buffer_starting = cv2.imencode('.jpg', blank_frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
    starting_bytes = buffer_starting.tobytes()

    error_frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), np.uint8)
    cv2.putText(error_frame, "NO SIGNAL", (20, STREAM_HEIGHT//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    _, buffer_error = cv2.imencode('.jpg', error_frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
    error_bytes = buffer_error.tobytes()

    # Initial frame
    current_frame = encoded_frame if encoded_frame else starting_bytes

    yield (b'--frame\r\n'
           b'Content-Type: image/jpeg\r\n\r\n' + current_frame + b'\r\n')
    
    while state.running:
        # Check if camera exists
        if state.camera is None or not state.camera.isOpened():
             time.sleep(1)
             yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + error_bytes + b'\r\n')
             continue

        with frame_condition:
            notified = frame_condition.wait(timeout=0.1)
            
            if not notified:
                # Timeout implies no new frames coming -> might be stuck
                pass

            if current_frame_id == last_sent_frame_id:
                # No new frame yet
                continue
            
            current_frame = encoded_frame
            last_sent_frame_id = current_frame_id

        if current_frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + current_frame + b'\r\n')


def generate_frames_right():
    global encoded_frame_right, current_frame_id_right
    
    current_frame = encoded_frame_right
    last_sent_frame_id = 0
    
    # Create fallback frames
    blank_frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), np.uint8)
    cv2.putText(blank_frame, "STARTING...", (20, STREAM_HEIGHT//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    _, buffer_starting = cv2.imencode('.jpg', blank_frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
    starting_bytes = buffer_starting.tobytes()

    error_frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), np.uint8)
    cv2.putText(error_frame, "NO SIGNAL (RIGHT)", (20, STREAM_HEIGHT//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    _, buffer_error = cv2.imencode('.jpg', error_frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
    error_bytes = buffer_error.tobytes()

    # Initial frame
    current_frame = encoded_frame_right if encoded_frame_right else starting_bytes

    yield (b'--frame\r\n'
           b'Content-Type: image/jpeg\r\n\r\n' + current_frame + b'\r\n')
    
    while state.running:
        # Check if right camera exists
        if state.camera_right is None or not state.camera_right.isOpened():
             time.sleep(1)
             yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + error_bytes + b'\r\n')
             continue

        with frame_condition_right:
            notified = frame_condition_right.wait(timeout=0.1)
            
            if not notified:
                pass
            
            if current_frame_id_right == last_sent_frame_id:
                continue
            
            current_frame = encoded_frame_right
            last_sent_frame_id = current_frame_id_right

        if current_frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + current_frame + b'\r\n')





def release_camera():
    """Release camera resources."""
    if state.camera:
        try:
            state.camera.release()
            print("✓ Camera released")
        except Exception as e:
            print(f"✗ Camera cleanup error: {e}")
    state.camera = None
    
    if state.camera_right:
        try:
            state.camera_right.release()
            print("✓ Right camera released")
        except Exception as e:
            print(f"✗ Right camera cleanup error: {e}")
    state.camera_right = None
