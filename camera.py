"""
ARCS - Camera Module
Handles camera initialization and MJPEG streaming for all 4 cameras:
  - Main (right arm), Right (left arm), Down (2D nadir), Fwd (depth/stereo)
"""

import time
import cv2
import numpy as np
import threading

from core.config_manager import get_config
from state import state

CAMERA_PORT = get_config("CAMERA_PORT")
CAMERA_WIDTH = get_config("CAMERA_WIDTH")
CAMERA_HEIGHT = get_config("CAMERA_HEIGHT")
CAMERA_BUFFER_SIZE = get_config("CAMERA_BUFFER_SIZE")
STREAM_WIDTH = get_config("STREAM_WIDTH")
STREAM_HEIGHT = get_config("STREAM_HEIGHT")
STREAM_JPEG_QUALITY = get_config("STREAM_JPEG_QUALITY")
CAMERA_RIGHT_PORT = get_config("CAMERA_RIGHT_PORT")
CAMERA_DOWN_PORT = get_config("CAMERA_DOWN_PORT")
CAMERA_FWD_PORT = get_config("CAMERA_FWD_PORT")


def _connect_camera_device(port, width, height, fps=30):
    print(f"📷 Connecting camera ({port})...", end=" ", flush=True)
    try:
        try:
            camera = cv2.VideoCapture(port, cv2.CAP_V4L2)
        except Exception:
            camera = cv2.VideoCapture(port)

        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        camera.set(cv2.CAP_PROP_FPS, fps)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if camera.isOpened():
            for _ in range(5):
                camera.grab()
            print("✓")
            return camera
        else:
            print("✗")
            return None

    except Exception as e:
        print(f"⚠ ({e})")
        return None


def init_camera() -> bool:
    try:
        if state.camera and state.camera.isOpened():
            print(f"📷 Camera ({CAMERA_PORT})... ✓ (Already open)")
        else:
            state.camera = _connect_camera_device(CAMERA_PORT, CAMERA_WIDTH, CAMERA_HEIGHT)
            if state.camera:
                threading.Thread(target=_capture_loop, daemon=True).start()

        if state.camera_right and state.camera_right.isOpened():
            print(f"📷 Camera Left Arm ({CAMERA_RIGHT_PORT})... ✓ (Already open)")
        else:
            if CAMERA_RIGHT_PORT:
                state.camera_right = _connect_camera_device(CAMERA_RIGHT_PORT, CAMERA_WIDTH, CAMERA_HEIGHT)
                if state.camera_right:
                    threading.Thread(target=_capture_loop_right, daemon=True).start()

        if state.camera_down and state.camera_down.isOpened():
            print(f"📷 Camera Down ({CAMERA_DOWN_PORT})... ✓ (Already open)")
        else:
            if CAMERA_DOWN_PORT:
                state.camera_down = _connect_camera_device(CAMERA_DOWN_PORT, CAMERA_WIDTH, CAMERA_HEIGHT)
                if state.camera_down:
                    threading.Thread(target=_capture_loop_down, daemon=True).start()

        if state.camera_fwd and state.camera_fwd.isOpened():
            print(f"📷 Camera Fwd Depth ({CAMERA_FWD_PORT})... ✓ (Already open)")
        else:
            if CAMERA_FWD_PORT:
                state.camera_fwd = _connect_camera_device(CAMERA_FWD_PORT, CAMERA_WIDTH, CAMERA_HEIGHT)
                if state.camera_fwd:
                    threading.Thread(target=_capture_loop_fwd, daemon=True).start()

        return True

    except Exception as e:
        print(f"✗ Failed: {e}")
        state.last_error = f"Camera init failed: {e}"
        return state.camera is not None


# --- Synchronization primitives ---

frame_condition = threading.Condition()
encoded_frame = None
current_frame_id = 0

frame_condition_right = threading.Condition()
encoded_frame_right = None
current_frame_id_right = 0

frame_condition_down = threading.Condition()
encoded_frame_down = None
current_frame_id_down = 0

frame_condition_fwd = threading.Condition()
encoded_frame_fwd = None
current_frame_id_fwd = 0


def _encode_and_notify(frame, condition, frame_ref_setter, id_setter):
    stream_frame = cv2.resize(frame, (STREAM_WIDTH, STREAM_HEIGHT), interpolation=cv2.INTER_LINEAR)
    _, buffer = cv2.imencode('.jpg', stream_frame, [
        cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY,
        cv2.IMWRITE_JPEG_OPTIMIZE, 0
    ])
    with condition:
        frame_ref_setter(buffer.tobytes())
        id_setter()
        condition.notify_all()


def _make_blank(label: str):
    frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), np.uint8)
    cv2.putText(frame, label, (20, STREAM_HEIGHT // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
    return buf.tobytes()


# --- Capture loops ---

def _capture_loop():
    global encoded_frame, current_frame_id
    print("[Camera] Right arm capture thread started")
    blank = _make_blank("WAITING...")
    with frame_condition:
        encoded_frame = blank
        frame_condition.notify_all()

    while state.running and state.camera and state.camera.isOpened():
        try:
            grabbed = state.camera.grab()
            if grabbed:
                ret, frame = state.camera.retrieve()
            else:
                ret, frame = False, None

            if ret and frame is not None:
                state.latest_frame = frame
                state.frame_id += 1
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
    print("[Camera] Right arm capture thread stopped")


def _capture_loop_right():
    global encoded_frame_right, current_frame_id_right
    print("[Camera] Left arm capture thread started")
    blank = _make_blank("WAITING...")
    with frame_condition_right:
        encoded_frame_right = blank
        frame_condition_right.notify_all()

    while state.running and state.camera_right and state.camera_right.isOpened():
        try:
            grabbed = state.camera_right.grab()
            if grabbed:
                ret, frame = state.camera_right.retrieve()
            else:
                ret, frame = False, None

            if ret and frame is not None:
                state.latest_frame_right = frame
                state.frame_id_right += 1
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
            print(f"[Camera] Left arm thread error: {e}")
            time.sleep(0.1)
    print("[Camera] Left arm capture thread stopped")


def _capture_loop_down():
    global encoded_frame_down, current_frame_id_down
    print("[Camera] Down capture thread started")
    blank = _make_blank("WAITING...")
    with frame_condition_down:
        encoded_frame_down = blank
        frame_condition_down.notify_all()

    while state.running and state.camera_down and state.camera_down.isOpened():
        try:
            grabbed = state.camera_down.grab()
            if grabbed:
                ret, frame = state.camera_down.retrieve()
            else:
                ret, frame = False, None

            if ret and frame is not None:
                state.latest_frame_down = frame
                state.frame_id_down += 1
                stream_frame = cv2.resize(frame, (STREAM_WIDTH, STREAM_HEIGHT), interpolation=cv2.INTER_LINEAR)
                _, buffer = cv2.imencode('.jpg', stream_frame, [
                    cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY,
                    cv2.IMWRITE_JPEG_OPTIMIZE, 0
                ])
                with frame_condition_down:
                    encoded_frame_down = buffer.tobytes()
                    current_frame_id_down = state.frame_id_down
                    frame_condition_down.notify_all()
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[Camera] Down thread error: {e}")
            time.sleep(0.1)
    print("[Camera] Down capture thread stopped")


def _capture_loop_fwd():
    global encoded_frame_fwd, current_frame_id_fwd
    print("[Camera] Fwd depth capture thread started")
    blank = _make_blank("WAITING...")
    with frame_condition_fwd:
        encoded_frame_fwd = blank
        frame_condition_fwd.notify_all()

    while state.running and state.camera_fwd and state.camera_fwd.isOpened():
        try:
            grabbed = state.camera_fwd.grab()
            if grabbed:
                ret, frame = state.camera_fwd.retrieve()
            else:
                ret, frame = False, None

            if ret and frame is not None:
                state.latest_frame_fwd = frame
                state.frame_id_fwd += 1
                stream_frame = cv2.resize(frame, (STREAM_WIDTH, STREAM_HEIGHT), interpolation=cv2.INTER_LINEAR)
                _, buffer = cv2.imencode('.jpg', stream_frame, [
                    cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY,
                    cv2.IMWRITE_JPEG_OPTIMIZE, 0
                ])
                with frame_condition_fwd:
                    encoded_frame_fwd = buffer.tobytes()
                    current_frame_id_fwd = state.frame_id_fwd
                    frame_condition_fwd.notify_all()
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[Camera] Fwd thread error: {e}")
            time.sleep(0.1)
    print("[Camera] Fwd depth capture thread stopped")


# --- Frame generators ---

def _make_error_frame(label: str):
    frame = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), np.uint8)
    cv2.putText(frame, label, (20, STREAM_HEIGHT // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY])
    return buffer.tobytes()


def _generator(camera_attr, condition_obj, encoded_ref, frame_id_ref, error_label):
    error_bytes = _make_error_frame(error_label)
    current = encoded_ref[0] if encoded_ref[0] else error_bytes
    last_id = 0

    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + current + b'\r\n')

    while state.running:
        cam = getattr(state, camera_attr)
        if cam is None or not cam.isOpened():
            time.sleep(1)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + error_bytes + b'\r\n')
            continue

        with condition_obj:
            condition_obj.wait(timeout=0.1)
            if frame_id_ref[0] == last_id:
                continue
            current = encoded_ref[0]
            last_id = frame_id_ref[0]

        if current:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + current + b'\r\n')


def generate_frames():
    global encoded_frame, current_frame_id
    error = _make_error_frame("NO SIGNAL")
    last_id = 0
    cur = encoded_frame if encoded_frame else error
    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + cur + b'\r\n')
    while state.running:
        if state.camera is None or not state.camera.isOpened():
            time.sleep(1)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + error + b'\r\n')
            continue
        with frame_condition:
            frame_condition.wait(timeout=0.1)
            if current_frame_id == last_id:
                continue
            cur = encoded_frame
            last_id = current_frame_id
        if cur:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + cur + b'\r\n')


def generate_frames_right():
    global encoded_frame_right, current_frame_id_right
    error = _make_error_frame("NO SIGNAL (LEFT ARM)")
    last_id = 0
    cur = encoded_frame_right if encoded_frame_right else error
    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + cur + b'\r\n')
    while state.running:
        if state.camera_right is None or not state.camera_right.isOpened():
            time.sleep(1)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + error + b'\r\n')
            continue
        with frame_condition_right:
            frame_condition_right.wait(timeout=0.1)
            if current_frame_id_right == last_id:
                continue
            cur = encoded_frame_right
            last_id = current_frame_id_right
        if cur:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + cur + b'\r\n')


def generate_frames_down():
    global encoded_frame_down, current_frame_id_down
    error = _make_error_frame("NO SIGNAL (DOWN)")
    last_id = 0
    cur = encoded_frame_down if encoded_frame_down else error
    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + cur + b'\r\n')
    while state.running:
        if state.camera_down is None or not state.camera_down.isOpened():
            time.sleep(1)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + error + b'\r\n')
            continue
        with frame_condition_down:
            frame_condition_down.wait(timeout=0.1)
            if current_frame_id_down == last_id:
                continue
            cur = encoded_frame_down
            last_id = current_frame_id_down
        if cur:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + cur + b'\r\n')


def generate_frames_fwd():
    global encoded_frame_fwd, current_frame_id_fwd
    error = _make_error_frame("NO SIGNAL (FWD DEPTH)")
    last_id = 0
    cur = encoded_frame_fwd if encoded_frame_fwd else error
    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + cur + b'\r\n')
    while state.running:
        if state.camera_fwd is None or not state.camera_fwd.isOpened():
            time.sleep(1)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + error + b'\r\n')
            continue
        with frame_condition_fwd:
            frame_condition_fwd.wait(timeout=0.1)
            if current_frame_id_fwd == last_id:
                continue
            cur = encoded_frame_fwd
            last_id = current_frame_id_fwd
        if cur:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + cur + b'\r\n')


def release_camera():
    for attr, label in [
        ('camera', 'Right arm camera'),
        ('camera_right', 'Left arm camera'),
        ('camera_down', 'Down camera'),
        ('camera_fwd', 'Fwd depth camera'),
    ]:
        cam = getattr(state, attr)
        if cam:
            try:
                cam.release()
                print(f"✓ {label} released")
            except Exception as e:
                print(f"✗ {label} cleanup error: {e}")
        setattr(state, attr, None)
