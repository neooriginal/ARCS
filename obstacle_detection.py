import cv2
import numpy as np
import logging
import threading
from collections import deque
from state import state

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
    
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        
        # Detection Thresholds
        # Y-coordinate thresholds (0=top, 480=bottom)
        self.obstacle_threshold_y = 420  # Stop when obstacle is very close
        self.center_x_threshold = 310
        self.min_edge_pixels = 200      # Minimum edge pixels to consider valid visual input
        
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
        
    def process(self, frame):
        """
        Process a video frame to detect obstacles and determine safe navigation actions.
        
        Args:
            frame (np.ndarray): Input video frame (BGR).
            
        Returns:
            tuple: (
                safe_actions (list): List of allowed actions ['FORWARD', 'LEFT', 'RIGHT', 'BACKWARD'],
                overlay (np.ndarray): Visualization frame,
                metrics (dict): Internal detection metrics
            )
        """
        if frame is None:
            return ["STOP"], None, {}

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
        center_width = len(edge_points) // 6
        side_width = (len(edge_points) - center_width) // 2
        
        c_left = self._get_chunk_average(edge_points[:side_width])
        c_fwd = self._get_chunk_average(edge_points[side_width : side_width + center_width])
        c_right = self._get_chunk_average(edge_points[side_width + center_width:])

        # 4. Check Safety Constraints
        is_blind = total_edge_pixels < self.min_edge_pixels
        instant_blocked = self._determine_blocked_directions(c_left, c_fwd, c_right, is_blind)
        
        # Update Safety History & Public State
        safe_actions = self._update_safety_state(instant_blocked, is_blind, shapes, overlay, w, h)

        # 5. Compute Precision Guidance (if enabled)
        guidance = ""
        recovery_hint = ""
        if state.precision_mode:
            guidance = self._compute_precision_guidance(edge_points, c_fwd, w, h, overlay, shapes)
            
            # Recovery/Wiggle Logic:
            # If we are blocked forward but in precision mode, maybe we just need to rotate?
            if "FORWARD" not in safe_actions:
                # Compare sides. If left is closer (higher Y), we should rotate right.
                # Threshold for "significant difference" to avoid noise
                diff = c_left - c_right
                if diff > 30: # Left is closer
                     recovery_hint = "OBSTACLE ON LEFT. ROTATE RIGHT TO UNSTICK."
                elif diff < -30: # Right is closer
                     recovery_hint = "OBSTACLE ON RIGHT. ROTATE LEFT TO UNSTICK."

        # Blend Visualization
        alpha = 0.4
        cv2.addWeighted(shapes, alpha, overlay, 1 - alpha, 0, overlay)
        if guidance:
            cv2.putText(overlay, guidance, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        return safe_actions, overlay, {
            'c_left': c_left, 
            'c_fwd': c_fwd, 
            'c_right': c_right, 
            'edges': total_edge_pixels,
            'edges': total_edge_pixels,
            'guidance': guidance,
            'recovery_hint': recovery_hint
        }

    def _detect_edges(self, frame):
        """Apply filters and Canny edge detection."""
        filtered = cv2.bilateralFilter(frame, 9, 75, 75)
        edges = cv2.Canny(filtered, 50, 150)
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

    def _get_chunk_average(self, chunk, top_n=2):
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
        """Determine which directions are unsafe based on thresholds."""
        blocked = set()
        
        # Adjust threshold based on mode
        threshold = self.obstacle_threshold_y
        if state.precision_mode:
             # Relax threshold to allow closer approach
             threshold += 30
             
        side_threshold = threshold + 50
        
        if is_blind:
            blocked.add("FORWARD")
        else:
            if state.precision_mode:
                 if c_fwd > 460:
                     blocked.add("FORWARD")
            else:
                 if c_fwd > threshold:
                     blocked.add("FORWARD")
                 if c_left > side_threshold:
                     blocked.add("LEFT")
                 if c_right > side_threshold:
                     blocked.add("RIGHT")
                
        return blocked

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
        VISION-FGM (Follow The Gap Method) with Safety Bubbles.
        1. Convert visual boundaries to "Free Space" array.
        2. "Inflate" obstacles (Safety Bubbles) to account for robot width.
        3. Find largest/deepest gap in non-inflated space.
        """
        # --- 1. Generate Pseudo-Scan (Free Space Profile) ---
        # Map x-coordinate to available depth (inv_y)
        # We process 'edge_points' which are sparse (step=5) into a dense array if needed,
        # but working with the sparse array is faster.
        
        # 'scan' will hold the Y-coordinate of the obstacle (High Y = Close Obstacle)
        # We want to minimize Y.
        scan = np.zeros(w, dtype=np.int32)
        
        # Fill scan with 0 (infinite depth) initially
        # Then populate with detected obstacles
        for x, y in edge_points:
            # Masking bottom threshold (Optional, keeping it for robustness)
            if y > 420:
                 y = 0
            
            # Simple interpolation/filling for sparse points
            # Fill a small kernel around the point to make it solid
            start = max(0, x - 2)
            end = min(w, x + 3)
            scan[start:end] = np.maximum(scan[start:end], y)

        # --- 2. Safety Bubble Inflation ---
        # If an obstacle is close (High Y), we must be far from it.
        # Robot Width approx 30-40cm. In pixels this varies by depth.
        # Simple heuristic: The "Close" zone (Y > 300) requires ~60px clearance radius.
        
        inflated_scan = scan.copy()
        
        # Iterate through scan to apply bubbles
        # This is O(W*R), optimization possible but W=640 is small.
        for x in range(0, w, 5): 
            y = scan[x]
            if y > 300: # Obstacle is "Close"
                # Radius increases with proximity
                # y=300 -> radius=30
                # y=450 -> radius=60
                radius = int(30 + (y - 300) * 0.2)
                
                left_bound = max(0, x - radius)
                right_bound = min(w, x + radius)
                
                # Mark inflated area as "Blocked" (Max Y)
                # We use 480 (Bottom) to signify "Blocked by Bubble"
                inflated_scan[left_bound:right_bound] = np.maximum(inflated_scan[left_bound:right_bound], 480)
                
                # Visualization: Draw bubbles
                cv2.circle(shapes, (x, y), radius, (0, 0, 255), 1)

        # --- 3. Find Deepest/Best Gap ---
        # A gap is a sequence where inflated_scan < PASSABLE_THRESHOLD (e.g. 350)
        PASSABLE_LIMIT_Y = 350
        
        gaps = []
        current_gap = []
        
        for x in range(w):
            if inflated_scan[x] < PASSABLE_LIMIT_Y:
                current_gap.append(x)
            else:
                if len(current_gap) > 20: # Min gap width ~20px
                    gaps.append(current_gap)
                current_gap = []
        if len(current_gap) > 20:
            gaps.append(current_gap)
            
        if not gaps:
            # FGM Fail-safe: "Blind Commit" if we are close but no gaps (likely inside door)
            if c_fwd > 400:
                 return "ALIGNMENT: FGM SAYS COMMIT. GO FORWARD."
            return "ALIGNMENT: BLOCKED. NO PATH."

        # --- 4. Select Best Gap ---
        image_center = w // 2
        
        def gap_score(gap):
            # Same scoring logic: Width - Penalty * DistToCenter
            width = len(gap)
            center = (gap[0] + gap[-1]) // 2
            dist = abs(center - image_center)
            
            # Use 'Last Gap' Hysteresis
            bonus = 0
            if self.last_gap_center is not None:
                if abs(center - self.last_gap_center) < 50:
                    bonus = 100
                    
            return width - (dist * 1.0) + bonus
            
        best_gap = max(gaps, key=gap_score)
        raw_gap_center = (best_gap[0] + best_gap[-1]) // 2
        
        # EMA Smoothing
        if self.last_gap_center is None:
            self.last_gap_center = raw_gap_center
        
        alpha = 0.3
        smoothed_center = int(alpha * raw_gap_center + (1 - alpha) * self.last_gap_center)
        self.last_gap_center = smoothed_center
        
        # --- 5. Visualization & Guidance ---
        # Draw dynamic green road
        path_poly = np.array([
            [int(w*0.2), h], 
            [int(w*0.8), h], 
            [smoothed_center + 40, int(h*0.5)], 
            [smoothed_center - 40, int(h*0.5)]
        ], np.int32)
        cv2.fillPoly(shapes, [path_poly], (0, 255, 0))
        
        cv2.line(overlay, (smoothed_center, h//2), (smoothed_center, h), (255, 255, 0), 3)
        cv2.putText(overlay, "FGM TARGET", (smoothed_center - 40, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Navigation Logic
        center_offset = smoothed_center - image_center
        if abs(center_offset) < 25:
            return "ALIGNMENT: ON PATH. GO FORWARD."
        elif center_offset < 0:
            return "ALIGNMENT: PATH LEFTSIDE. Turn LEFT."
        else:
            return "ALIGNMENT: PATH RIGHTSIDE. Turn RIGHT."
