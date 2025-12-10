import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)

class ObstacleDetector:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        # Thresholds tuned for "Stop at the very last moment"
        # Y=480 is bottom. Y=420 is very close.
        self.obstacle_threshold_y = 420
        self.center_x_threshold = 310
        
        # Hysteresis / Flicker Safety
        # Store recent "Blocked" states.
        # If an action was blocked recently, we keep it blocked for a bit.
        from collections import deque
        import threading
        self.history_len = 12 # Approx 0.5-1.0s depending on FPS
        self.block_history = deque(maxlen=self.history_len) # Stores set of BLOCKED actions
        self.lock = threading.Lock()
        
    def process(self, frame):
        """
        Process the frame to detect obstacles and determine navigation command.
        Returns:
            safe_actions (list): List of allowed actions ['FORWARD', 'LEFT', 'RIGHT', 'BACKWARD']
            overlay (np.ndarray): Frame with debug drawing
            metrics (dict): Internal metrics
        """
        if frame is None:
            return ["STOP"], None, {}

        # Noise Reduction
        filtered = cv2.bilateralFilter(frame, 9, 75, 75)
        
        # Edge Detection
        edges = cv2.Canny(filtered, 50, 150)
        
        h, w = edges.shape
        edge_points = []
        
        # Visualization setup
        overlay = frame.copy()
        shapes = frame.copy()
        
        # Column Scan
        for x in range(0, w, 5):
            detected_y = 0 
            found = False
            for y in range(h - 1, -1, -1):
                if edges[y, x] == 255:
                    detected_y = y
                    found = True
                    break
            edge_points.append((x, detected_y))
            if found:
                cv2.circle(overlay, (x, detected_y), 2, (0, 0, 255), -1)

        # Chunking & Metrics
        num_points = len(edge_points)
        # Narrower Forward Zone for Doors (1/6th instead of 1/5th)
        center_width = num_points // 6
        side_width = (num_points - center_width) // 2
        
        left_chunk = edge_points[:side_width]
        center_chunk = edge_points[side_width : side_width + center_width]
        right_chunk = edge_points[side_width + center_width:]
        
        # Grid Visualization
        center_x_start = side_width * 5
        center_x_end = (side_width + center_width) * 5
        cv2.rectangle(shapes, (center_x_start, 0), (center_x_end, h), (50, 50, 50), 1)
        
        def get_top_average(chunk, top_n=2):
            """
            Get the average of the Top N closest points.
            Robustly detects thin obstacles (like cables with 2 edges)
            while filtering single-line noise.
            """
            if not chunk: return 0
            ys = sorted([p[1] for p in chunk], reverse=True) # Descending (Closest first)
            top_values = ys[:top_n]
            if not top_values: return 0
            return sum(top_values) / len(top_values)
            
        c_left = get_top_average(left_chunk)
        c_fwd = get_top_average(center_chunk)
        c_right = get_top_average(right_chunk)

        # Safety Logic
        
        # Determine INSTANT blocked actions
        instant_blocked = set()
        threshold = self.obstacle_threshold_y
        
        # --- BLINDNESS CHECK ---
        total_edge_pixels = np.count_nonzero(edges)
        min_edge_pixels = 200 
        is_blind = total_edge_pixels < min_edge_pixels
        
        if is_blind:
            instant_blocked.add("FORWARD")
            cv2.rectangle(shapes, (int(w*0.2), int(h*0.2)), (int(w*0.8), int(h*0.8)), (0, 0, 255), -1)
            cv2.putText(overlay, "BLOCKED (NO VISUALS)", (int(w*0.3), int(h*0.5)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            if c_fwd > threshold:
                instant_blocked.add("FORWARD")
                cv2.rectangle(shapes, (int(w*0.33), int(h*0.5)), (int(w*0.66), h), (0, 0, 255), -1)
            
            side_threshold = threshold + 50
            if c_left > side_threshold:
                instant_blocked.add("LEFT")
                cv2.rectangle(shapes, (0, int(h*0.5)), (int(w*0.33), h), (0, 0, 255), -1)
            
            if c_right > side_threshold:
                instant_blocked.add("RIGHT")
                cv2.rectangle(shapes, (int(w*0.66), int(h*0.5)), (w, h), (0, 0, 255), -1)

        # Update History (Thread Safe)
        with self.lock:
            self.block_history.append(instant_blocked)
            
            # Calculate PERSISTENT blocked actions
            persistent_blocked = set()
            for b_set in self.block_history:
                persistent_blocked.update(b_set)
            
        # Determine Safe Actions based on persistent_blocked
        safe_actions = ["BACKWARD"]
        
        if "FORWARD" not in persistent_blocked:
             safe_actions.append("FORWARD")
             if not is_blind:
                  pts = np.array([[int(w*0.3), h], [int(w*0.7), h], [int(w*0.6), int(h*0.4)], [int(w*0.4), int(h*0.4)]], np.int32)
                  cv2.fillPoly(shapes, [pts], (0, 255, 0))
                  cv2.putText(overlay, "FWD OK", (int(w*0.45), int(h*0.8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                 
        if "LEFT" not in persistent_blocked:
            safe_actions.append("LEFT")
            if not is_blind:
                 pts = np.array([[0, h], [0, h//2], [w//3, h//2], [0, h]], np.int32)
                 # cv2.fillPoly for left zone if desired
                 
        if "RIGHT" not in persistent_blocked:
            safe_actions.append("RIGHT")
            
        # Blend overlay
        alpha = 0.4
        cv2.addWeighted(shapes, alpha, overlay, 1 - alpha, 0, overlay)
        
        # Gap Detection
        # Find the "Center of Safety" to guide the robot
        # We classify columns as "Passable" if their obstacle is far away (e.g. Y < 350)
        # We classify columns as "Passable" if their obstacle is far away (e.g. Y < 350)
        # Smoothing: Filter out single-column noise spikes by checking neighbors
        raw_ys = [p[1] for p in edge_points]
        smoothed_ys = []
        for i in range(len(raw_ys)):
            # Median of 3 neighbors
            prev_y = raw_ys[i-1] if i > 0 else raw_ys[i]
            next_y = raw_ys[i+1] if i < len(raw_ys)-1 else raw_ys[i]
            curr_y = raw_ys[i]
            smoothed_ys.append(sorted([prev_y, curr_y, next_y])[1])

        passable_indices = []
        for i, (x, _) in enumerate(edge_points):
            # Use smoothed Y for check
            if smoothed_ys[i] < 350: 
                passable_indices.append(x)
                
        guidance = "" # Default to empty if no gap found
        best_center_x = w // 2 # Default to center
        
        if passable_indices:
            # Find the largest contiguous segment of passable columns
            # But edge_points is sparse (step 5). 
            # We can simplify: just find the mean X of all passable points?
            # No, that might put us between two doors.
            # We need the widest gap.
            
            # Group into clusters
            # Since step is 5, if diff > 10, it's a break.
            clusters = []
            if passable_indices:
                current_cluster = [passable_indices[0]]
                for i in range(1, len(passable_indices)):
                    # Tolerant of single noise pixel (5) or double noise pixels (10). 
                    # Step is 5. Adjacent is 5. Gap of 1 is 10. Gap of 2 is 15.
                    if passable_indices[i] - passable_indices[i-1] <= 15:
                        current_cluster.append(passable_indices[i])
                    else:
                        clusters.append(current_cluster)
                        current_cluster = [passable_indices[i]]
                clusters.append(current_cluster)
                
            # Find widest cluster
            widest_cluster = max(clusters, key=len)
            
            # Calculate center of widest gap
            if widest_cluster:
                start_x = widest_cluster[0]
                end_x = widest_cluster[-1]
                gap_center = (start_x + end_x) // 2
                best_center_x = gap_center
                
                # Draw Gap Target
                cv2.line(overlay, (gap_center, h//2), (gap_center, h), (255, 255, 0), 2)
                cv2.putText(overlay, "TARGET", (gap_center - 20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                # Calculate simple guidance
                center_offset = gap_center - (w // 2)
                # > 0 means Target is Right. < 0 means Target is Left.
                
                if abs(center_offset) < 40:
                    guidance = "ALIGNMENT: PERFECT. Go straight."
                elif center_offset < 0:
                    guidance = f"ALIGNMENT: Gap is LEFT. Turn LEFT slightly."
                else:
                    guidance = f"ALIGNMENT: Gap is RIGHT. Turn RIGHT slightly."
        
        if guidance:
             cv2.putText(overlay, guidance, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        return safe_actions, overlay, {
            'c_left': c_left, 'c_fwd': c_fwd, 'c_right': c_right, 'edges': total_edge_pixels,
            'guidance': guidance
        }

