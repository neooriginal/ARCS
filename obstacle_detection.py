
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
        self.approach_stop_distance = get_config("LIDAR_APPROACH_DISTANCE", 2)
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
        3. Analyze low-image for "Ground/Low Obstacles".
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
        
        instant_blocked = set()
        
        if distance is None:
            status = "NO SIGNAL" if state.lidar is not None else "DISCONNECTED"
            self._draw_no_lidar(overlay, w, h, status)
        else:
            self.distance_history.append(distance)
            avg_distance = sum(self.distance_history) / len(self.distance_history)
            
            # Use approach distance if active
            current_stop_dist = self.approach_stop_distance if state.approach_mode else self.stop_distance
            
            if avg_distance < current_stop_dist:
                instant_blocked.add("FORWARD")
            
            self._draw_proximity_overlay(overlay, avg_distance, w, h)
        
        # Check low obstacles if not in approach mode
        vision_blocked = False
        if not state.approach_mode:
            vision_blocked = self._check_visual_obstacles(frame, overlay)
            if vision_blocked:
                instant_blocked.add("FORWARD")
        
        safe_actions = self._update_safety_state(instant_blocked)
        
        guidance = ""
        rotation_hint = None
        target_x = -1
        
        if state.precision_mode:
            target_x, guidance = self._find_visual_gap(frame, w, h)
            if target_x != -1:
                self._draw_target_guidance(overlay, target_x, w, h)
        
        # --- 4. SCAN RESULT OVERLAY ---
        # --- 4. SCAN RESULT OVERLAY ---
        if state.last_scan_result:
            self._draw_scan_result(overlay, state.last_scan_result, w, h)
        
        self._draw_mode_status(overlay, w, h)
        
        if vision_blocked and "FORWARD" in instant_blocked:
             cv2.putText(overlay, "BLOCKED: LOW OBSTACLE", (w//2 - 100, h - 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
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

    def _check_visual_obstacles(self, frame, overlay):
        """Check for obstacles using edge density in lower center region."""
        h, w = frame.shape[:2]
        roi_y = int(h * 0.65)
        roi = frame[roi_y:h, :]
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        center_x = w // 2
        check_w = int(w * 0.4)
        center_roi = edges[:, center_x - check_w//2 : center_x + check_w//2]
        
        edge_density = np.mean(center_roi) / 255.0
        
        # High edge density suggests complex object
        is_blocked = edge_density > 0.08
        
        if is_blocked:
            p1 = (center_x - check_w//2, roi_y)
            p2 = (center_x + check_w//2, h)
            cv2.rectangle(overlay, p1, p2, (0, 0, 255), 2)
            
        return is_blocked

    def _find_visual_gap(self, frame, w, h):
        """Find path of least resistance using intensity variance."""
        roi = frame[self.scan_height_start:self.scan_height_end, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Gaussian blur to remove noise
        gray = cv2.GaussianBlur(gray, (15, 15), 0)
        
        # Analyze columns (axis 0 = vertical in ROI)
        col_means = np.mean(gray, axis=0)
        col_vars = np.var(gray, axis=0)
        
        # Normalize scores
        norm_means = col_means / 255.0
        norm_vars = col_vars / np.max(col_vars) if np.max(col_vars) > 0 else col_vars
        
        # Combined score: weight smoothness higher than brightness
        scores = (norm_means * 0.4) + (norm_vars * 0.6)
        
        kernel_size = 50
        scores_smooth = np.convolve(scores, np.ones(kernel_size)/kernel_size, mode='same')
        
        # Find minimum score index
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
        # Calculate thresholds based on mode for display color
        limit = self.approach_stop_distance if state.approach_mode else self.stop_distance
        
        if distance <= limit:
            color = (0, 0, 255) # Red
            text = "STOP"
        elif distance <= self.warn_distance and not state.approach_mode:
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

    def _draw_no_lidar(self, overlay, w, h, status="DISCONNECTED"):
        cv2.rectangle(overlay, (0, 0), (w, 30), (50, 50, 50), -1)
        cv2.putText(overlay, f"LIDAR: {status}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)

    def _draw_scan_result(self, overlay, result, w, h):
        """Draw the LIDAR scan profile and detected gap."""
        if not result:
            return

        # 1. Draw Background Box
        box_h = 100
        box_y = h - 150
        cv2.rectangle(overlay, (10, box_y), (w-10, box_y + box_h), (0, 0, 0), -1)
        cv2.rectangle(overlay, (10, box_y), (w-10, box_y + box_h), (50, 50, 50), 1)
        
        if not result.get('found', False):
            cv2.putText(overlay, f"SCAN FAILED: {result.get('reason')}", (20, box_y + 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return

        # 2. Draw Graph
        raw = result.get('raw_profile')
        if not raw:
            return
            
        angles, dists = raw
        if len(dists) == 0:
            return
            
        # Scaling
        # Angles: -25 to +25 mapped to 10 to w-10
        # Dists: 0 to 200cm mapped to box_h to 0 (inverted)
        min_angle = angles.min()
        max_angle = angles.max()
        angle_range = max_angle - min_angle if max_angle != min_angle else 1
        
        max_dist = 200.0 # Clip at 2m
        
        points = []
        for a, d in zip(angles, dists):
            x = 10 + int(((a - min_angle) / angle_range) * (w - 20))
            y = box_y + box_h - int(min(d, max_dist) / max_dist * box_h)
            points.append((x, y))
            
        if len(points) > 1:
            cv2.polylines(overlay, [np.array(points)], False, (0, 255, 255), 2)
            
        # 3. Draw Gap Markers
        # Left Edge
        left_a = result['left_edge_angle']
        lx = 10 + int(((left_a - min_angle) / angle_range) * (w - 20))
        cv2.line(overlay, (lx, box_y), (lx, box_y + box_h), (0, 0, 255), 2)
        
        # Right Edge
        right_a = result['right_edge_angle']
        rx = 10 + int(((right_a - min_angle) / angle_range) * (w - 20))
        cv2.line(overlay, (rx, box_y), (rx, box_y + box_h), (0, 0, 255), 2)
        
        # Center Target
        center_a = result['center_angle']
        cx = 10 + int(((center_a - min_angle) / angle_range) * (w - 20))
        cv2.line(overlay, (cx, box_y), (cx, box_y + box_h), (0, 255, 0), 2)
        
        # Text Info
        info = f"GAP: {result['width_deg']:.1f}deg CENTER: {center_a:.1f}deg"
        cv2.putText(overlay, info, (20, box_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    def _draw_mode_status(self, overlay, w, h):
        mode_text = "MODE: STANDARD"
        color = (0, 255, 0)
        
        if state.approach_mode:
            mode_text = "MODE: APPROACH (UNSAFE - 2cm)"
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
