
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
        self.visual_block_history = deque(maxlen=6)  # History for visual obstacle smoothing
        self.lock = threading.Lock()
        
        self.latest_blockage = {
            'forward': False,
            'left': False,
            'right': False
        }
        
        self.last_frame_id = -1
        self.cached_result = (["STOP"], None, {})

    def _get_camera_right_frame(self):
        """Helper to get right camera frame safely."""
        # Check if right camera is active and has a frame
        if state.camera_right and state.latest_frame_right is not None:
             return state.latest_frame_right
        return None
        
    def process(self, frame):
        """
        """Process frame for safety checks and obstacle detection."""
        """
        if frame is None:
            return ["STOP"], None, {}
            
        with self.lock:
            # Cache check
            if state.frame_id == self.last_frame_id:
                return self.cached_result
        
        # 360° LIDAR Safety Check
        distance = state.lidar_distance
        lidar = state.lidar360
        h, w = frame.shape[:2]
        overlay = frame.copy()
        
        instant_blocked = set()
        
        if distance is None:
            status = "NO SIGNAL" if state.lidar360 is not None else "DISCONNECTED"
            self._draw_no_lidar(overlay, w, h, status)
        else:
            self.distance_history.append(distance)
            avg_distance = sum(self.distance_history) / len(self.distance_history)
            
            # Use approach distance if active
            current_stop_dist = self.approach_stop_distance if state.approach_mode else self.stop_distance
            
            # --- 360° LIDAR SAFETY BUBBLE ---
            if lidar and lidar.connected:
                fwd_min = lidar.get_min_distance_in_range(-30, 30)
                if fwd_min is not None and fwd_min < current_stop_dist:
                    instant_blocked.add("FORWARD")
                elif avg_distance < current_stop_dist:
                    instant_blocked.add("FORWARD")
                    
                back_min = lidar.get_min_distance_in_range(150, 210)
                if back_min is not None and back_min < current_stop_dist:
                    instant_blocked.add("BACKWARD")
                    
                left_min = lidar.get_min_distance_in_range(60, 120)
                if left_min is not None and left_min < 35:
                    instant_blocked.add("LEFT")
                
                right_min = lidar.get_min_distance_in_range(240, 300)
                if right_min is not None and right_min < 35:
                    instant_blocked.add("RIGHT")
            else:
                # Fallback to single point if 360 not ready
                if avg_distance < current_stop_dist:
                    instant_blocked.add("FORWARD")
            
            self._draw_proximity_overlay(overlay, avg_distance, w, h)
        
        # Check low obstacles if not in approach mode or precision mode
        # Precision mode disables visual check because door frames/thresholds cause false positives
        visual_blocks = set()
        if not state.approach_mode and not state.precision_mode:
            visual_blocks = self._check_visual_obstacles(frame, overlay)
            
            # PASSIVE MODE: Visual blocks do NOT stop movement due to 2m blind spot.
            # They serve as warnings only.
            # instant_blocked.update(visual_blocks)  <-- DISABLED
            
            # Check Right Camera
            right_frame = self._get_camera_right_frame()
            if right_frame is not None:
                if self._check_right_camera_obstacle(right_frame):
                    # Right camera also passive
                    # instant_blocked.add("RIGHT") 
                    # Draw warning on main overlay
                    cv2.putText(overlay, "RIGHT CAM: ALERT", (w - 220, 80), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        safe_actions = self._update_safety_state(instant_blocked)
        
        guidance = ""
        rotation_hint = None
        target_x = -1
        
        # In precision mode, don't use visual gap detection - it's unreliable
        # AI should use find_gap() for LIDAR-based gap detection and alignment
        if state.precision_mode:
            guidance = "USE find_gap() FOR GAP DETECTION"
        
        # --- 4. SCAN RESULT OVERLAY ---
        if state.last_scan_result:
            self._draw_scan_result(overlay, state.last_scan_result, w, h)
        
        self._draw_mode_status(overlay, w, h)
        
        if len(visual_blocks) > 0:
             cv2.putText(overlay, "VISUAL ALERT: LOW OBSTACLE", (w//2 - 120, h - 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
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
        """
        Check for obstacles using gradient magnitude (Sobel).
        Splits view into LEFT, CENTER, RIGHT zones.
        Returns a set of blocked directions.
        """
        h, w = frame.shape[:2]
        
        # ROI: Lower 40% of image
        roi_y = int(h * 0.60) 
        roi = frame[roi_y:h, :]
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Larger blur kernel to reduce floor texture/shadow noise
        blur = cv2.GaussianBlur(gray, (9, 9), 0)
        
        # Use Sobel
        sobelx = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=3)
        magnitude = cv2.magnitude(sobelx, sobely)
        
        # Get threshold
        sobel_thresh = get_config("OBSTACLE_SOBEL_THRESHOLD", 55)
        
        # Create binary mask of strong edges
        _, mask = cv2.threshold(magnitude, sobel_thresh, 255, cv2.THRESH_BINARY)
        
        # Visualization of edges
        if np.any(mask):
            edge_layer = np.zeros((roi.shape[0], roi.shape[1], 3), dtype=np.uint8)
            edge_layer[mask > 0] = (255, 255, 0) # Cyan
            
            full_layer = np.zeros_like(overlay)
            full_layer[roi_y:h, :] = edge_layer

            cv2.addWeighted(overlay, 1.0, full_layer, 0.5, 0, overlay)

        # Zone Definitions
        zone_w = w // 3
        zones = {
            "LEFT":  (0, zone_w),
            "FORWARD": (zone_w, 2 * zone_w),
            "RIGHT": (2 * zone_w, w)
        }
        
        blocked_directions = set()
        density_threshold = get_config("OBSTACLE_DENSITY_THRESHOLD", 0.05)

        for direction, (start_x, end_x) in zones.items():
            zone_roi = mask[:, start_x:end_x]
            if zone_roi.size == 0: continue
            
            edge_density = np.count_nonzero(zone_roi) / zone_roi.size
            
            # Simple temporal smoothing could be added per-zone here if needed.
            # For now, using direct density.
            
            if edge_density > density_threshold:
                blocked_directions.add(direction)
                
                # Draw Box on Overlay
                p1 = (start_x, roi_y)
                p2 = (end_x, h)
                cv2.rectangle(overlay, p1, p2, (0, 0, 255), 2)
                cv2.putText(overlay, f"{direction} BLOCKED", (start_x + 10, roi_y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        return blocked_directions

    def _check_right_camera_obstacle(self, frame):
        """Simple check for obstacles in the right camera view."""
        # Using similar logic to main camera but full width since it's a side view
        h, w = frame.shape[:2]
        roi = frame[int(h*0.5):, :] # Look at bottom half
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 0)
        
        sobelx = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=3)
        magnitude = cv2.magnitude(sobelx, sobely)
        
        sobel_thresh = get_config("OBSTACLE_SOBEL_THRESHOLD", 55)
        _, mask = cv2.threshold(magnitude, sobel_thresh, 255, cv2.THRESH_BINARY)
        
        # Check center-ish area of right cam
        center_roi = mask[:, int(w*0.2):int(w*0.8)]
        density = np.count_nonzero(center_roi) / center_roi.size
        
        return density > get_config("OBSTACLE_DENSITY_THRESHOLD", 0.05)

    def _find_visual_gap(self, frame, w, h):
        """Find path of least resistance by looking for the DARKEST column (gap = dark)."""
        roi = frame[self.scan_height_start:self.scan_height_end, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Heavy blur to smooth out floor patterns
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        # Analyze columns - gaps appear DARK (light doesn't reflect back from far away)
        col_means = np.mean(gray, axis=0)
        
        # Smooth the column scores
        kernel_size = 60
        scores_smooth = np.convolve(col_means, np.ones(kernel_size)/kernel_size, mode='same')
        
        # Restrict search to center 40%
        margin = int(w * 0.3)
        center_scores = scores_smooth.copy()
        center_scores[:margin] = 255  # High = not a gap
        center_scores[-margin:] = 255
        
        # Strong center bias - prefer center when no clear gap
        center_x = w // 2
        center_bias = np.abs(np.arange(w) - center_x) / (w / 2) * 30  # Up to 30 brightness units bias
        center_scores = center_scores + center_bias
        
        # Find DARKEST column (minimum brightness = likely gap)
        best_x = np.argmin(center_scores)
        
        # Calculate offset from center
        offset = best_x - center_x
        
        guidance = ""
        threshold_pixels = 40  # Reduced threshold for guidance
        
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
            # Don't overlay error - it's already logged
            return

        # 2. Draw Graph
        raw = result.get('raw_profile')
        if not raw:
            return
            
        angles, dists = raw
        if len(dists) == 0:
            return
            
        # Convert to numpy arrays for vector ops
        angles = np.array(angles)
        dists = np.array(dists)
            
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
            
            # Check blockage for each direction
            # Require > 50% frames in history to have the block to trigger it (structural temporal smoothing)
            threshold = len(self.block_history) / 2
            
            forward_blocked = sum(1 for b in self.block_history if "FORWARD" in b) > threshold
            backward_blocked = sum(1 for b in self.block_history if "BACKWARD" in b) > threshold
            left_blocked = sum(1 for b in self.block_history if "LEFT" in b) > threshold
            right_blocked = sum(1 for b in self.block_history if "RIGHT" in b) > threshold
            
            self.latest_blockage = {
                'forward': forward_blocked,
                'backward': backward_blocked,
                'left': left_blocked,
                'right': right_blocked
            }
            
        allowed_actions = []
        if not forward_blocked: allowed_actions.append("FORWARD")
        if not backward_blocked: allowed_actions.append("BACKWARD")
        if not left_blocked: allowed_actions.append("LEFT")
        if not right_blocked: allowed_actions.append("RIGHT")
        
        return allowed_actions
