

import cv2
import numpy as np
import logging
import threading
from collections import deque
from state import state
from core.config_manager import get_config

CAMERA_WIDTH = get_config("CAMERA_WIDTH")
CAMERA_HEIGHT = get_config("CAMERA_HEIGHT")

logger = logging.getLogger(__name__)


class ObstacleDetector:
    def __init__(self, width=None, height=None):
        self.width = width or CAMERA_WIDTH
        self.height = height or CAMERA_HEIGHT
        
        # LIDAR Thresholds
        self.stop_distance = get_config("LIDAR_STOP_DISTANCE", 30)
        self.warn_distance = get_config("LIDAR_WARN_DISTANCE", 80)
        self.max_display_distance = get_config("LIDAR_MAX_DISPLAY", 200)
        
        # Visual Gap Finding
        self.scan_height_start = int(self.height * 0.4)
        self.scan_height_end = int(self.height * 0.6)
        
        self.history_len = 8
        self.distance_history = deque(maxlen=self.history_len)
        self.block_history = deque(maxlen=self.history_len)
        self.lock = threading.Lock()
        
        self.latest_blockage = {
            'forward': False,
            'left': False,
            'right': False
        }
        
        self.last_frame_id = -1
        self.cached_result = (["STOP"], None, {})
        
    def process(self, frame):
        """
        Process frame.
        1. Read LIDAR for safety (Stop/Go).
        2. Analyze image for "Gap" (Guidance).
        """
        if frame is None:
            return ["STOP"], None, {}
            
        with self.lock:
            # Cache check
            if state.frame_id == self.last_frame_id:
                return self.cached_result
        
        distance = state.lidar_distance
        h, w = frame.shape[:2]
        overlay = frame.copy()
        
        # --- 1. LIDAR SAFETY CHECK ---
        instant_blocked = set()
        avg_distance = 0
        
        if distance is None:
            self._draw_no_lidar(overlay, w, h)
        else:
            self.distance_history.append(distance)
            avg_distance = sum(self.distance_history) / len(self.distance_history)
            
            if avg_distance < self.stop_distance:
                instant_blocked.add("FORWARD")
            
            self._draw_proximity_overlay(overlay, avg_distance, w, h)
        
        safe_actions = self._update_safety_state(instant_blocked)
        
        # --- 2. VISUAL GAP FINDING (Guidance) ---
        guidance = ""
        rotation_hint = None
        target_x = -1
        
        if state.precision_mode:
            target_x, guidance = self._find_visual_gap(frame, w, h)
            if target_x != -1:
                self._draw_target_guidance(overlay, target_x, w, h)
        
        # Draw Mode
        self._draw_mode_status(overlay, w, h)
        
        metrics = {
            'distance': distance,
            'guidance': guidance,
            'rotation_hint': rotation_hint
        }
        
        result = (safe_actions, overlay, metrics)
        
        with self.lock:
            self.last_frame_id = state.frame_id
            self.cached_result = result
            
        return result

    def _find_visual_gap(self, frame, w, h):
        """
        Simple heuristic to find Open Space:
        Look for the 'smoothest' (low variance) and 'darkest' (low intensity) 
        vertical column in the center strip.
        """
        # Crop center strip
        roi = frame[self.scan_height_start:self.scan_height_end, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Gaussian blur to remove noise
        gray = cv2.GaussianBlur(gray, (15, 15), 0)
        
        # Analyze columns (axis 0 = vertical in ROI)
        # We want columns with LOW standard deviation (smooth floor/space)
        # and LOW mean brightness (often depth/shadows in indoor hallways)
        col_means = np.mean(gray, axis=0)
        col_vars = np.var(gray, axis=0)
        
        # Normalize scores
        norm_means = col_means / 255.0
        norm_vars = col_vars / np.max(col_vars) if np.max(col_vars) > 0 else col_vars
        
        # Combined score: We want minimal score
        # Weight smoothness (variance) higher than brightness
        scores = (norm_means * 0.4) + (norm_vars * 0.6)
        
        # Smooth scores horizontally
        kernel_size = 50
        scores_smooth = np.convolve(scores, np.ones(kernel_size)/kernel_size, mode='same')
        
        # Find minimum score index (Best Gap)
        best_x = np.argmin(scores_smooth)
        
        # Calculate offset from center
        center_x = w // 2
        offset = best_x - center_x
        
        guidance = ""
        threshold_pixels = 50
        
        if abs(offset) > threshold_pixels:
            if offset < 0:
                guidance = "ROTATE LEFT"
            else:
                guidance = "ROTATE RIGHT"
        else:
            guidance = "FORWARD CLEAR"
            
        return best_x, guidance

    def _draw_proximity_overlay(self, overlay, distance, w, h):
        """Draw top bar showing LIDAR distance."""
        if distance <= self.stop_distance:
            color = (0, 0, 255) # Red
            text = "STOP"
        elif distance <= self.warn_distance:
            color = (0, 165, 255) # Orange
            text = "CAUTION"
        else:
            color = (0, 255, 0) # Green
            text = "CLEAR"
            
        # Draw Bar Background
        bar_h = 30
        cv2.rectangle(overlay, (0, 0), (w, bar_h), (40, 40, 40), -1)
        
        # Fill percentage
        ratio = min(1.0, distance / self.max_display_distance)
        fill_w = int(w * ratio)
        cv2.rectangle(overlay, (0, 0), (fill_w, bar_h), color, -1)
        
        # Text
        display_text = f"LIDAR: {int(distance)}cm - {text}"
        cv2.putText(overlay, display_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Draw forward safety zone
        self._draw_safety_zone(overlay, distance, w, h, color)

    def _draw_safety_zone(self, overlay, distance, w, h, color):
        """Projected safety trapezoid on floor."""
        overlay_layer = overlay.copy()
        pts = np.array([
            [int(w*0.2), h], 
            [int(w*0.8), h], 
            [int(w*0.6), int(h*0.6)], 
            [int(w*0.4), int(h*0.6)]
        ], np.int32)
        
        cv2.fillPoly(overlay_layer, [pts], color)
        cv2.addWeighted(overlay_layer, 0.3, overlay, 0.7, 0, overlay)

    def _draw_target_guidance(self, overlay, target_x, w, h):
        """Draw visual target line for gap."""
        # Vertical Target Line
        cv2.line(overlay, (target_x, int(h*0.4)), (target_x, h), (255, 255, 0), 2)
        cv2.circle(overlay, (target_x, int(h*0.7)), 5, (255, 255, 0), -1)
        cv2.putText(overlay, "GAP", (target_x - 15, int(h*0.4)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    def _draw_no_lidar(self, overlay, w, h):
        cv2.rectangle(overlay, (0, 0), (w, 30), (50, 50, 50), -1)
        cv2.putText(overlay, "LIDAR DISCONNECTED", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

    def _draw_mode_status(self, overlay, w, h):
        mode_text = "MODE: STANDARD"
        color = (0, 255, 0)
        
        if state.approach_mode:
            mode_text = "MODE: APPROACH (UNSAFE)"
            color = (0, 0, 255)
        elif state.precision_mode:
            mode_text = "MODE: PRECISION (GAP FINDING)"
            color = (255, 255, 0)
            
        cv2.putText(overlay, mode_text, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def _update_safety_state(self, instant_blocked):
        with self.lock:
            self.block_history.append(instant_blocked)
            # Filter noise (require > 50% frames to be blocked)
            count = sum(1 for b in self.block_history if "FORWARD" in b)
            is_blocked = count > (len(self.block_history) / 2)
            
            self.latest_blockage['forward'] = is_blocked
            
        actions = ["LEFT", "RIGHT", "BACKWARD"]
        if not is_blocked:
            actions.append("FORWARD")
        return actions
