import cv2
import numpy as np
import logging
import threading
from collections import deque
import time
from state import state
from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT, 
    OBSTACLE_SOBEL_THRESHOLD, 
    OBSTACLE_THRESHOLD_RATIO
)

logger = logging.getLogger(__name__)

class ObstacleDetector:
    """
    Vision-based obstacle detection and navigation assistance system.
    
    Features:
    - Vertical edge detection for obstacle identification.
    - Dynamic safety thresholds for different movement modes.
    - "Precision Mode" for aligning with narrow gaps/doors.
    - Continuous safety history to prevent flickering/hysteresis.
    """
    
    def __init__(self, width=None, height=None):
        self.width = width or CAMERA_WIDTH
        self.height = height or CAMERA_HEIGHT
        
        # Detection Thresholds
        # Calculated based on resolution and config
        self.obstacle_threshold_y = int(self.height * OBSTACLE_THRESHOLD_RATIO)
        self.approach_threshold_y = int(self.height * 0.98)     # ~470 @ 480
        
        # Precision Mode Thresholds
        self.precision_fwd_limit = int(self.height * 0.96)      # ~460 @ 480
        self.precision_padding = int(self.height * 0.0625)      # ~30 @ 480
        self.precision_align_limit = int(self.height * 0.92)    # ~440 @ 480
        
        # Gap Detection Thresholds
        self.passable_limit_y = int(self.height * 0.73)         # ~350 @ 480
        self.side_padding = int(self.height * 0.105)            # ~50 @ 480
        
        # Horizontal Tolerances
        self.min_edge_pixels = int(200 * (self.width / 640.0))
        self.min_gap_width = int(20 * (self.width / 640.0))
        self.align_tolerance = int(20 * (self.width / 640.0))
        
        # Hysteresis / Safety History
        self.history_len = 12  # Approx 0.5-1.0s buffer
        self.block_history = deque(maxlen=self.history_len)
        self.lock = threading.Lock()
        
        # Public State
        self.latest_blockage = {
            'forward': False,
            'left': False,
            'right': False
        }
        
        # EMA Smoothing for Precision Mode
        self.last_gap_center = None
        self.last_gap_update_time = 0
        self.gap_lock_duration = 5.0  # seconds
        
        # Caching
        self.last_frame_id = -1
        self.cached_result = (["STOP"], None, {})
        
    def process(self, frame):
        """
        Process a video frame to detect obstacles and determine safe navigation actions.
        Uses caching to avoid double-processing the same frame.
        """
        if frame is None:
            return ["STOP"], None, {}
            
        with self.lock:
            # Check Cache
            if state.frame_id == self.last_frame_id:
                return self.cached_result

        # 1. Image Preprocessing & Edge Detection
        edges, total_edge_pixels = self._detect_edges(frame)
        h, w = edges.shape
        
        # 2. Column Scanning
        edge_points = self._scan_columns(edges, w, h)
        
        # Visualization Setup
        overlay = frame.copy()
        shapes = frame.copy()
        self._draw_scan_points(overlay, edge_points)

        # 3. Analyze Obstacle Distances
        # Divide view into chunks: Left, Center, Right
        # Center is narrower to focus on immediate path.
        center_width = len(edge_points) // 5
        side_width = (len(edge_points) - center_width) // 2
        
        c_left = self._get_chunk_average(edge_points[:side_width])
        c_fwd = self._get_chunk_average(edge_points[side_width : side_width + center_width])
        c_right = self._get_chunk_average(edge_points[side_width + center_width:])

        # 4. Check Safety Constraints
        is_blind = total_edge_pixels < self.min_edge_pixels
        instant_blocked, rotation_hint = self._determine_blocked_directions(c_left, c_fwd, c_right, is_blind)
        
        # Update Safety History & Public State
        safe_actions = self._update_safety_state(instant_blocked, is_blind, shapes, overlay, w, h)

        # 5. Compute Precision Guidance (if enabled)
        guidance = ""
        if state.precision_mode:
            guidance = self._compute_precision_guidance(edge_points, c_fwd, w, h, overlay, shapes)

        # Blend Visualization
        alpha = 0.4
        cv2.addWeighted(shapes, alpha, overlay, 1 - alpha, 0, overlay)
        if guidance:
            cv2.putText(overlay, guidance, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Draw Mode Status
        mode_text = "MODE: STANDARD"
        mode_color = (0, 255, 0) # Green
        
        if state.approach_mode:
            mode_text = "MODE: APPROACH (SAFETY OFF)"
            mode_color = (0, 0, 255) # Red
        elif state.precision_mode:
            mode_text = "MODE: PRECISION"
            mode_color = (255, 255, 0) # Cyan
            
        cv2.putText(overlay, mode_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)

        result = (safe_actions, overlay, {
            'c_left': c_left, 
            'c_fwd': c_fwd, 
            'c_right': c_right, 
            'edges': total_edge_pixels,
            'guidance': guidance,
            'rotation_hint': rotation_hint
        })
        
        with self.lock:
            self.last_frame_id = state.frame_id
            self.cached_result = result
            
        return result

    def _detect_edges(self, frame):
        """
        Detect vertical edges using Sobel-X operator.
        This ignores horizontal lines (like carpet/floor textures) and highlights vertical obstacles.
        """
        # Grey scale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Blur slightly to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Sobel-X: Gradient in X direction (detects vertical lines)
        sobelx = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
        abs_sobelx = np.absolute(sobelx)
        
        # Threshold
        _, edges = cv2.threshold(abs_sobelx, OBSTACLE_SOBEL_THRESHOLD, 255, cv2.THRESH_BINARY)
        edges = np.uint8(edges)
        
        # Morphological Open to remove noise (small specks)
        kernel = np.ones((3,3), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel)
        
        total_pixels = np.count_nonzero(edges)
        return edges, total_pixels

    def _scan_columns(self, edges, w, h, step=5):
        """Scan columns to find the lowest (closest) edge pixel."""
        edge_points = []
        for x in range(0, w, step):
            detected_y = 0
            # Scan bottom-up
            for y in range(h - 1, -1, -1):
                if edges[y, x] == 255:
                    detected_y = y
                    break
            edge_points.append((x, detected_y))
        return edge_points

    def _draw_scan_points(self, overlay, edge_points):
        """Draw detected obstacles on the overlay."""
        for x, y in edge_points:
            if y > 0:
                cv2.circle(overlay, (x, y), 2, (0, 0, 255), -1)

    def _get_chunk_average(self, chunk, top_n=10):
        """
        Calculate average Y-position of the closest points in a chunk.
        Robust against single-pixel noise.
        """
        if not chunk:
            return 0
        ys = sorted([p[1] for p in chunk], reverse=True)  # Descending (Closest first)
        top_values = ys[:top_n]
        if not top_values:
            return 0
        return sum(top_values) / len(top_values)

    def _determine_blocked_directions(self, c_left, c_fwd, c_right, is_blind):
        """Determine which directions are unsafe based on thresholds.
        Returns: (blocked_set, rotation_hint)
        """
        blocked = set()
        rotation_hint = None
        
        threshold = self.obstacle_threshold_y
        if state.precision_mode:
             threshold += self.precision_padding
        
        # In Approach Mode, we allow obstacles to come VERY close (bottom of screen)
        if state.approach_mode:
             threshold = self.approach_threshold_y
             
        side_threshold = threshold + self.side_padding
        
        if is_blind:
            blocked.add("FORWARD")
        else:
            if state.precision_mode:
                 if c_fwd > self.precision_fwd_limit:
                     blocked.add("FORWARD")
                     # Provide rotation hint based on clearance
                     if c_left < c_right:
                         rotation_hint = "ROTATE LEFT to align"
                     elif c_right < c_left:
                         rotation_hint = "ROTATE RIGHT to align"
            
            elif state.approach_mode:
                # APPROACH MODE: DISABLE FORWARD SAFETY
                # We want to touch objects. Only check sides.
                pass 
                
            else:
                 if c_fwd > threshold:
                     blocked.add("FORWARD")
                 if c_left > side_threshold:
                     blocked.add("LEFT")
                 if c_right > side_threshold:
                     blocked.add("RIGHT")
                
        return blocked, rotation_hint

    def _update_safety_state(self, instant_blocked, is_blind, shapes, overlay, w, h):
        """
        Update shared history buffer and determine final safe actions.
        Draws safety indicators on the overlay.
        """
        with self.lock:
            self.block_history.append(instant_blocked)
            
            # Combine history to filter noise
            persistent_blocked = set()
            for b_set in self.block_history:
                persistent_blocked.update(b_set)
            
            # Update public state
            self.latest_blockage = {
                'forward': "FORWARD" in persistent_blocked,
                'left': "LEFT" in persistent_blocked,
                'right': "RIGHT" in persistent_blocked
            }
            
        safe_actions = ["BACKWARD"] # Backward is mostly always safe (blind)
        
        # Visualize Blockages
        cx = w // 2
        cy = h // 2
        
        if is_blind:
            cv2.rectangle(shapes, (int(w*0.2), int(h*0.2)), (int(w*0.8), int(h*0.8)), (0, 0, 255), -1)
            cv2.putText(overlay, "BLOCKED (NO VISUALS)", (int(w*0.3), cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            if "FORWARD" in persistent_blocked:
                cv2.rectangle(shapes, (int(w*0.33), cy), (int(w*0.66), h), (0, 0, 255), -1)
            else:
                safe_actions.append("FORWARD")
                # Draw Safe Zone
                pts = np.array([[int(w*0.3), h], [int(w*0.7), h], [int(w*0.6), int(h*0.4)], [int(w*0.4), int(h*0.4)]], np.int32)
                cv2.fillPoly(shapes, [pts], (0, 255, 0))
                cv2.putText(overlay, "FWD OK", (int(w*0.45), int(h*0.8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            if "LEFT" in persistent_blocked:
                cv2.rectangle(shapes, (0, cy), (int(w*0.33), h), (0, 0, 255), -1)
            else:
                safe_actions.append("LEFT")
                
            if "RIGHT" in persistent_blocked:
                cv2.rectangle(shapes, (int(w*0.66), cy), (w, h), (0, 0, 255), -1)
            else:
                safe_actions.append("RIGHT")
                
        return safe_actions

    def _compute_precision_guidance(self, edge_points, c_fwd, w, h, overlay, shapes):
        """
        Identify usable gaps and provide alignment guidance.
        """
        # 1. Smooth Y-values to reduce noise
        raw_ys = [p[1] for p in edge_points]
        smoothed_ys = []
        for i in range(len(raw_ys)):
            prev_y = raw_ys[i-1] if i > 0 else raw_ys[i]
            next_y = raw_ys[i+1] if i < len(raw_ys)-1 else raw_ys[i]
            smoothed_ys.append(sorted([prev_y, raw_ys[i], next_y])[1]) # Median
            
        # 2. Identify "Passable" Columns (Obstacle is far away)
        is_very_close = c_fwd > self.obstacle_threshold_y
        
        # CLOSE-RANGE BYPASS: When very close, gap detection is unreliable
        if is_very_close:
            return "BLIND COMMIT: Decide based on what you see."
        
        passable_indices = []
        for i, y in enumerate(smoothed_ys):
             effective_y = y
             if state.precision_mode and y > self.obstacle_threshold_y:
                 effective_y = 0 
                 
             if effective_y < self.passable_limit_y:
                 passable_indices.append(edge_points[i][0])
        
        if not passable_indices:
            return ""

        # 3. Find Largest Contiguous Gap
        # Points are separated by 'step=5'. Allow skip of 1-2 points (approx 15px)
        clusters = []
        current_cluster = [passable_indices[0]]
        for i in range(1, len(passable_indices)):
            if passable_indices[i] - passable_indices[i-1] <= 8:
                current_cluster.append(passable_indices[i])
            else:
                clusters.append(current_cluster)
                current_cluster = [passable_indices[i]]
        clusters.append(current_cluster)
        
        # Filter small gaps (noise) and find cluster closest to center
        # Minimum gap width approx 20px (scaled)
        valid_clusters = [c for c in clusters if (c[-1] - c[0]) > self.min_gap_width]
        
        if not valid_clusters:
            return ""
            
        # Smart Gap Selection: Score = Width - (DistanceToCenter * Weight)
        # We want wide gaps, but we PENALIZE gaps far from the center.
        image_center = w // 2
        
        def gap_score(cluster):
            width = cluster[-1] - cluster[0]
            center = (cluster[0] + cluster[-1]) // 2
            dist = abs(center - image_center)
            # Weight: 1.0 means 1px of distance cancels 1px of width. 
            # Lower weight (0.5) means we prefer width more. Higher (2.0) means we prefer center more.
            # Using 1.2 to slightly bias towards center over raw width.
            return width - (dist * 1.2)
            
        best_cluster = max(valid_clusters, key=gap_score)
        
        raw_gap_center = (best_cluster[0] + best_cluster[-1]) // 2
        
        # Time-based smoothing: only update position every 5 seconds
        current_time = time.time()
        
        if self.last_gap_center is None:
            self.last_gap_center = raw_gap_center
            self.last_gap_update_time = current_time
        else:
            time_since_update = current_time - self.last_gap_update_time
            if time_since_update >= self.gap_lock_duration:
                # Lock expired, allow update with EMA
                alpha = 0.5
                self.last_gap_center = int(alpha * raw_gap_center + (1 - alpha) * self.last_gap_center)
                self.last_gap_update_time = current_time
                
        gap_center = self.last_gap_center
        
        # Draw Target Line
        cv2.line(overlay, (gap_center, h//2), (gap_center, h), (255, 255, 0), 2)
        cv2.putText(overlay, "TARGET", (gap_center - 20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # 4. Generate Guidance
        center_offset = gap_center - (w // 2)
        is_aligned = abs(center_offset) < self.align_tolerance
        is_too_close_to_align = c_fwd > self.precision_align_limit
        
        if is_aligned:
            cv2.line(overlay, (gap_center, h//2), (gap_center, h), (0, 255, 0), 3)
            return ""
        else:
            if is_too_close_to_align:
                cv2.rectangle(shapes, (0, 0), (w, h), (0, 0, 255), 20)
                return ""
            elif center_offset < 0:
                cv2.line(overlay, (gap_center, h//2), (gap_center, h), (0, 0, 255), 2)
                return ""
            else:
                cv2.line(overlay, (gap_center, h//2), (gap_center, h), (0, 0, 255), 2)
                return ""
